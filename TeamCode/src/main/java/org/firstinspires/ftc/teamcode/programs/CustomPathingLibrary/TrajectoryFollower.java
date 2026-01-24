package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Advanced Trajectory Follower with:
 * - Frenet frame error decomposition (tangent/normal to path)
 * - SPECIAL HANDLING for turn-in-place (uses world frame when v≈0)
 * - Feedforward from trajectory (velocity, acceleration, curvature)
 * - Feedback PID on position and heading errors
 * - Pure pursuit fallback for recovery when far from path
 * - Adaptive lookahead based on velocity
 * - Robust settling with timeout protection
 * - Slew rate limiting for smooth commands
 * - Disturbance rejection and path re-acquisition
 */
public class TrajectoryFollower {

    // ==================== INTERFACES ====================

    @FunctionalInterface
    public interface PoseSupplier {
        Pose2d getPose();
    }

    // ==================== CONFIGURATION ====================

    // Feedforward coefficients
    private double kFFVel = 1.0;      // Velocity feedforward gain
    private double kFFAccel = 0.0;    // Acceleration feedforward gain
    private double kFFOmega = 1.0;    // Angular velocity feedforward gain
    private double kFFAlpha = 0.05;   // Angular acceleration feedforward gain (seconds)

    // Cross-coupling gains
    private double kCrossTrackToOmega = 0.0;  // Convert lateral error to heading correction

    // Pure pursuit
    private double lookaheadBase = DriveConstants.LOOKAHEAD_DISTANCE;
    private double lookaheadMin = DriveConstants.MIN_LOOKAHEAD;
    private double lookaheadMax = DriveConstants.MAX_LOOKAHEAD;
    private double lookaheadVelScale = 0.15;  // lookahead = base + vel * scale

    // Recovery thresholds
    private double purePursuitThreshold = 6.0;  // Use pure pursuit if cross-track error exceeds this (in)
    private double maxRecoverySpeed = 30.0;     // Max speed during recovery (in/s)

    // Turn-in-place detection threshold
    private static final double TURN_IN_PLACE_VEL_THRESHOLD = 2.0;  // in/s

    // Turn-in-place position holding gain reduction factor
    // During turns, we use softer position gains to prevent overcorrection
    private static final double TURN_POSITION_GAIN_FACTOR = 0.15;  // 15% of normal gains

    // ==================== COMPONENTS ====================

    private final PoseSupplier poseSupplier;
    private final MecanumDrive drive;

    // PID controllers
    private final AdvancedPIDF pidLong;  // Longitudinal (along-path) error
    private final AdvancedPIDF pidLat;   // Lateral (cross-track) error
    private final AdvancedPIDF pidH;     // Heading error

    // Slew rate limiters
    private final SlewRateLimiter slewVx;
    private final SlewRateLimiter slewVy;
    private final SlewRateLimiter slewOmega;

    // ==================== STATE ====================

    public Trajectory traj = null;
    private double t0 = 0.0;           // Trajectory start time
    private double lastUpdateTime = 0.0;

    // End state cache
    public Trajectory.State endState = null;

    // Settle state machine
    private enum FollowState { TRACKING, RECOVERING, SETTLING, FINISHED }
    private FollowState state = FollowState.FINISHED;

    private double settleStartTime = 0.0;
    private Pose2d settleTarget = null;

    // Telemetry data
    private double lastLongError = 0, lastLatError = 0, lastHeadingError = 0;
    private double lastVxCmd = 0, lastVyCmd = 0, lastOmegaCmd = 0;
    private boolean wasInRecovery = false;

    // ==================== CONSTRUCTOR ====================

    public TrajectoryFollower(PoseSupplier supplier, MecanumDrive drive) {
        this.poseSupplier = supplier;
        this.drive = drive;

        // Initialize PID controllers with tuned gains
        pidLong = new AdvancedPIDF(DriveConstants.KP_X, DriveConstants.KI_X,
                DriveConstants.KD_X, DriveConstants.KF_X);
        pidLat = new AdvancedPIDF(DriveConstants.KP_Y, DriveConstants.KI_Y,
                DriveConstants.KD_Y, DriveConstants.KF_Y);
        pidH = new AdvancedPIDF(DriveConstants.KP_H, DriveConstants.KI_H,
                DriveConstants.KD_H, DriveConstants.KF_H);

        // Configure derivative filters
        double loopHz = DriveConstants.LOOP_HZ;
        double cutoffHz = DriveConstants.D_CUTOFF_HZ;
        pidLong.setDerivativeFilter(cutoffHz, loopHz);
        pidLat.setDerivativeFilter(cutoffHz, loopHz);
        pidH.setDerivativeFilter(cutoffHz, loopHz);

        // Configure integral limits and zones
        pidLong.setIntegralLimit(5.0);
        pidLong.setIntegralZone(3.0);  // Only integrate when close

        pidLat.setIntegralLimit(5.0);
        pidLat.setIntegralZone(2.0);

        pidH.setIntegralLimit(2.0);
        pidH.setIntegralZone(Math.toRadians(15));

        // Configure output limits
        pidLong.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidLat.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidH.setOutputLimits(-DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);

        // Configure error deadbands
        pidLong.setErrorDeadband(0.1);  // in
        pidLat.setErrorDeadband(0.1);   // in
        pidH.setErrorDeadband(Math.toRadians(0.5));  // rad

        // Initialize slew rate limiters
        slewVx = new SlewRateLimiter(DriveConstants.MAX_DVX_IN_S2);
        slewVy = new SlewRateLimiter(DriveConstants.MAX_DVY_IN_S2);
        slewOmega = new SlewRateLimiter(DriveConstants.MAX_DW_RAD_S2);
    }

    // ==================== CONFIGURATION METHODS ====================

    public TrajectoryFollower setVelocityFeedforward(double kV) {
        this.kFFVel = Math.max(0, kV);
        return this;
    }

    public TrajectoryFollower setAccelFeedforward(double kA) {
        this.kFFAccel = Math.max(0, kA);
        return this;
    }

    public TrajectoryFollower setOmegaFeedforward(double kOmega) {
        this.kFFOmega = Math.max(0, kOmega);
        return this;
    }

    public TrajectoryFollower setAlphaFeedforward(double kAlphaSec) {
        this.kFFAlpha = Math.max(0, kAlphaSec);
        return this;
    }

    public TrajectoryFollower setCrossTrackToHeadingGain(double k) {
        this.kCrossTrackToOmega = Math.max(0, k);
        return this;
    }

    public TrajectoryFollower setLookahead(double base, double min, double max) {
        this.lookaheadBase = base;
        this.lookaheadMin = min;
        this.lookaheadMax = max;
        return this;
    }

    public TrajectoryFollower setPurePursuitThreshold(double threshold) {
        this.purePursuitThreshold = Math.max(1.0, threshold);
        return this;
    }

    public TrajectoryFollower setRecoverySpeed(double maxSpeed) {
        this.maxRecoverySpeed = Math.max(5.0, maxSpeed);
        return this;
    }

    /**
     * Update PID gains dynamically (for live tuning).
     */
    public void setGains(double kpX, double kiX, double kdX,
                         double kpY, double kiY, double kdY,
                         double kpH, double kiH, double kdH) {
        pidLong.setGains(kpX, kiX, kdX, 0);
        pidLat.setGains(kpY, kiY, kdY, 0);
        pidH.setGains(kpH, kiH, kdH, 0);
    }

    // ==================== TRAJECTORY MANAGEMENT ====================

    /**
     * Set a new trajectory to follow.
     *
     * @param trajectory  The trajectory to follow
     * @param nowSec  Current time in seconds
     * @param alignToNearest  If true, find closest point on path and start there
     */
    public void setTrajectory(Trajectory trajectory, double nowSec, boolean alignToNearest) {
        this.traj = trajectory;
        this.lastUpdateTime = nowSec;

        // Reset controllers
        pidLong.reset();
        pidLat.reset();
        pidH.reset();
        slewVx.reset(0);
        slewVy.reset(0);
        slewOmega.reset(0);

        // Reset state
        state = FollowState.TRACKING;
        settleStartTime = 0;
        settleTarget = null;
        wasInRecovery = false;

        if (trajectory != null && !trajectory.allStates().isEmpty()) {
            // Cache end state
            endState = trajectory.allStates().get(trajectory.allStates().size() - 1);
            settleTarget = endState.pose;

            // Determine start time
            if (alignToNearest) {
                double closestT = trajectory.closestTimeTo(poseSupplier.getPose());
                t0 = nowSec - closestT;
            } else {
                t0 = nowSec;
            }
        } else {
            endState = null;
            t0 = nowSec;
        }
    }

    /**
     * Cancel current trajectory and stop.
     */
    public void cancel() {
        traj = null;
        state = FollowState.FINISHED;
        drive.setPowers(0, 0, 0, 0);
    }

    /**
     * Check if trajectory following is complete.
     */
    public boolean isFinished(double nowSec) {
        if (traj == null) return true;
        if (state == FollowState.FINISHED) return true;

        // Time-based completion
        double elapsed = nowSec - t0;
        if (elapsed >= traj.duration() + DriveConstants.SETTLE_TIMEOUT) {
            return true;
        }

        return false;
    }

    /**
     * Check if robot has settled at endpoint.
     */
    public boolean isSettled() {
        return state == FollowState.FINISHED;
    }

    // ==================== MAIN UPDATE LOOP ====================

    /**
     * Call every control loop iteration.
     */
    public void update(double nowSec) {
        // Calculate dt
        double dt = Math.max(0.001, nowSec - lastUpdateTime);
        lastUpdateTime = nowSec;

        // Handle null trajectory
        if (traj == null) {
            drive.setPowers(0, 0, 0, 0);
            state = FollowState.FINISHED;
            return;
        }

        // Get current pose
        Pose2d currentPose = poseSupplier.getPose();
        double elapsed = nowSec - t0;

        // State machine
        switch (state) {
            case TRACKING:
                updateTracking(currentPose, elapsed, dt);
                break;

            case RECOVERING:
                updateRecovery(currentPose, elapsed, dt);
                break;

            case SETTLING:
                updateSettling(currentPose, nowSec, dt);
                break;

            case FINISHED:
                drive.setPowers(0, 0, 0, 0);
                break;
        }
    }

    // ==================== TRACKING STATE ====================

    private void updateTracking(Pose2d currentPose, double elapsed, double dt) {
        // Get reference state from trajectory
        Trajectory.State ref = traj.sample(elapsed);

        // Calculate errors in world frame
        double dx = ref.pose.x - currentPose.x;
        double dy = ref.pose.y - currentPose.y;
        double distanceError = Math.hypot(dx, dy);

        // Heading error (shortest angle)
        double headingError = normalizeAngle(ref.pose.heading - currentPose.heading);

        // ============================================================
        // DETECT TURN-IN-PLACE MODE
        // When ref.v is near zero, we're doing a turn-in-place.
        // In this case, use WORLD FRAME for position correction,
        // not the rotating Frenet frame.
        // ============================================================
        boolean isTurnInPlace = Math.abs(ref.v) < TURN_IN_PLACE_VEL_THRESHOLD;

        double vxRobot, vyRobot, omegaCmd;

        if (isTurnInPlace) {
            // ==================== TURN-IN-PLACE MODE ====================
            // ONLY correct heading - NO position correction at all
            // Position drift during turns is acceptable and will be fixed after the turn

            // Store errors for telemetry (in world frame)
            lastLongError = dx;
            lastLatError = dy;
            lastHeadingError = headingError;

            // --- FEEDFORWARD ---
            double vComp = DriveConstants.getVoltageCompFactor();
            double omegaFF = ref.omega * kFFOmega + ref.alpha * kFFAlpha;
            omegaFF *= vComp;

            // --- FEEDBACK ---
            // ONLY heading feedback - ZERO position correction
            double omegaFB = pidH.update(0, -headingError, 0, dt);

            // NO position correction during turns
            vxRobot = 0.0;
            vyRobot = 0.0;

            // Combine feedforward and feedback for omega
            omegaCmd = omegaFF + omegaFB;

        } else {
            // ==================== NORMAL PATH FOLLOWING MODE ====================
            // Use Frenet frame (tangent/normal to path)

            // Decompose error into tangent/normal frame
            double tx = ref.tangent.x;
            double ty = ref.tangent.y;
            double tMag = Math.hypot(tx, ty);
            if (tMag < 1e-6) {
                tx = Math.cos(ref.pose.heading);
                ty = Math.sin(ref.pose.heading);
            } else {
                tx /= tMag;
                ty /= tMag;
            }
            double nx = -ty;
            double ny = tx;

            double longError = dx * tx + dy * ty;   // Along path
            double latError = dx * nx + dy * ny;    // Cross track

            // Switch to recovery if too far from path
            if (Math.abs(latError) > purePursuitThreshold) {
                state = FollowState.RECOVERING;
                wasInRecovery = true;
                updateRecovery(currentPose, elapsed, dt);
                return;
            }

            // Store for telemetry
            lastLongError = longError;
            lastLatError = latError;
            lastHeadingError = headingError;

            // --- FEEDFORWARD ---
            double vComp = DriveConstants.getVoltageCompFactor();

            // Velocity feedforward (along path tangent in world frame)
            double vffWorld_x = ref.v * tx * kFFVel;
            double vffWorld_y = ref.v * ty * kFFVel;

            // Acceleration feedforward
            vffWorld_x += ref.a * tx * kFFAccel;
            vffWorld_y += ref.a * ty * kFFAccel;

            // Angular feedforward
            double omegaFF = ref.omega * kFFOmega + ref.alpha * kFFAlpha;

            // Apply voltage compensation to feedforward
            vffWorld_x *= vComp;
            vffWorld_y *= vComp;
            omegaFF *= vComp;

            // --- FEEDBACK ---
            // PID on longitudinal error (along path)
            double vLongFB = pidLong.update(0, -longError, 0, dt);

            // PID on lateral error (cross track)
            double vLatFB = pidLat.update(0, -latError, 0, dt);

            // PID on heading error
            double omegaFB = pidH.update(0, -headingError, 0, dt);

            // Cross-track to heading coupling (helps robot point toward path)
            omegaFB += kCrossTrackToOmega * latError;

            // --- COMBINE FEEDFORWARD + FEEDBACK ---
            // Feedback is in Frenet frame, convert to world
            double vFB_world_x = vLongFB * tx + vLatFB * nx;
            double vFB_world_y = vLongFB * ty + vLatFB * ny;

            // Total world-frame velocity command
            double vxWorld = vffWorld_x + vFB_world_x;
            double vyWorld = vffWorld_y + vFB_world_y;
            omegaCmd = omegaFF + omegaFB;

            // --- CONVERT TO ROBOT FRAME ---
            double cos = Math.cos(currentPose.heading);
            double sin = Math.sin(currentPose.heading);
            vxRobot = vxWorld * cos + vyWorld * sin;
            vyRobot = -vxWorld * sin + vyWorld * cos;
        }

        // --- APPLY LIMITS ---
        // Slew rate limiting
        vxRobot = slewVx.filter(vxRobot, dt);
        vyRobot = slewVy.filter(vyRobot, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        // Velocity magnitude limiting
        double vMag = Math.hypot(vxRobot, vyRobot);
        if (vMag > DriveConstants.MAX_VEL_IN_S) {
            double scale = DriveConstants.MAX_VEL_IN_S / vMag;
            vxRobot *= scale;
            vyRobot *= scale;
        }

        // Angular velocity limiting
        omegaCmd = clamp(omegaCmd, -DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);

        // Store for telemetry
        lastVxCmd = vxRobot;
        lastVyCmd = vyRobot;
        lastOmegaCmd = omegaCmd;

        // --- OUTPUT TO DRIVE ---
        double[] powers = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxRobot, vyRobot, omegaCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, powers
        );
        drive.setPowers(powers[0], powers[1], powers[2], powers[3]);

        // --- CHECK FOR SETTLING ---
        boolean pastEnd = elapsed >= traj.duration();
        if (pastEnd) {
            state = FollowState.SETTLING;
            settleStartTime = lastUpdateTime;
        }
    }

    // ==================== RECOVERY STATE (PURE PURSUIT) ====================

    private void updateRecovery(Pose2d currentPose, double elapsed, double dt) {
        // Find closest point on trajectory
        double closestT = traj.closestTimeTo(currentPose);

        // Calculate adaptive lookahead
        Trajectory.State closestState = traj.sample(closestT);
        double lookahead = lookaheadBase + closestState.v * lookaheadVelScale;
        lookahead = clamp(lookahead, lookaheadMin, lookaheadMax);

        // Get lookahead point
        double lookaheadT = Math.min(closestT + lookahead / Math.max(1, closestState.v), traj.duration());
        Trajectory.State targetState = traj.sample(lookaheadT);

        // Vector to target
        double dx = targetState.pose.x - currentPose.x;
        double dy = targetState.pose.y - currentPose.y;
        double dist = Math.hypot(dx, dy);

        // Check if we've rejoined the path
        // Project error onto path normal
        double tx = targetState.tangent.x;
        double ty = targetState.tangent.y;
        double tMag = Math.hypot(tx, ty);
        if (tMag > 1e-6) { tx /= tMag; ty /= tMag; }
        double nx = -ty, ny = tx;
        double latError = dx * nx + dy * ny;

        if (Math.abs(latError) < purePursuitThreshold * 0.5) {
            // Rejoined path, return to tracking
            state = FollowState.TRACKING;
            // Re-sync time to current path position
            t0 = lastUpdateTime - closestT;
            return;
        }

        // Pure pursuit: drive toward target point
        double targetHeading = Math.atan2(dy, dx);
        double headingError = normalizeAngle(targetHeading - currentPose.heading);

        // Desired heading for target
        double desiredEndHeading = targetState.pose.heading;
        double endHeadingError = normalizeAngle(desiredEndHeading - currentPose.heading);

        // Speed proportional to distance, capped
        double speed = Math.min(maxRecoverySpeed, dist * 2.0);
        speed = Math.max(5.0, speed);  // Minimum recovery speed

        // World-frame velocity toward target
        double vxWorld = (dist > 0.1) ? (dx / dist) * speed : 0;
        double vyWorld = (dist > 0.1) ? (dy / dist) * speed : 0;

        // Angular velocity to correct heading
        double omegaCmd = pidH.update(0, -headingError * 0.5 - endHeadingError * 0.5, 0, dt);

        // Convert to robot frame
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);
        double vxRobot = vxWorld * cos + vyWorld * sin;
        double vyRobot = -vxWorld * sin + vyWorld * cos;

        // Apply slew limiting
        vxRobot = slewVx.filter(vxRobot, dt);
        vyRobot = slewVy.filter(vyRobot, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        // Store for telemetry
        lastVxCmd = vxRobot;
        lastVyCmd = vyRobot;
        lastOmegaCmd = omegaCmd;
        lastLatError = latError;

        // Output
        double[] powers = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxRobot, vyRobot, omegaCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, powers
        );
        drive.setPowers(powers[0], powers[1], powers[2], powers[3]);

        // Check for timeout
        if (elapsed > traj.duration() + DriveConstants.SETTLE_TIMEOUT) {
            state = FollowState.SETTLING;
            settleStartTime = lastUpdateTime;
        }
    }

    // ==================== SETTLING STATE ====================

    private void updateSettling(Pose2d currentPose, double nowSec, double dt) {
        if (settleTarget == null) {
            state = FollowState.FINISHED;
            drive.setPowers(0, 0, 0, 0);
            return;
        }

        // Calculate errors to target
        double dx = settleTarget.x - currentPose.x;
        double dy = settleTarget.y - currentPose.y;
        double posError = Math.hypot(dx, dy);
        double headingError = normalizeAngle(settleTarget.heading - currentPose.heading);

        // Detect if this was a turn-in-place trajectory
        // Check if trajectory had minimal TRANSLATION (not velocity, since end velocity is always 0)
        // A turn-in-place has the same start and end position
        boolean wasTurnInPlace = false;
        if (traj != null && !traj.allStates().isEmpty()) {
            Trajectory.State startState = traj.allStates().get(0);
            Trajectory.State endState = traj.allStates().get(traj.allStates().size() - 1);
            double trajDistance = Math.hypot(
                    endState.pose.x - startState.pose.x,
                    endState.pose.y - startState.pose.y
            );
            // If trajectory moved less than 2 inches total, it's a turn-in-place
            wasTurnInPlace = trajDistance < 2.0;
        }

        // Check if settled
        // For turn-in-place, use looser position tolerance since position wasn't the goal
        double posTolerance = wasTurnInPlace ?
                DriveConstants.END_POSITION_TOLERANCE * 3.0 :  // 3x tolerance for turns
                DriveConstants.END_POSITION_TOLERANCE;

        boolean posOK = posError < posTolerance;
        boolean headOK = Math.abs(headingError) < DriveConstants.END_HEADING_TOLERANCE;

        if (posOK && headOK) {
            double settleTime = nowSec - settleStartTime;
            if (settleTime >= DriveConstants.SETTLE_TIME) {
                state = FollowState.FINISHED;
                drive.setPowers(0, 0, 0, 0);
                return;
            }
        } else {
            // Reset settle timer if we leave the window
            settleStartTime = nowSec;
        }

        // Check for timeout
        double totalSettleTime = nowSec - settleStartTime;
        if (totalSettleTime > DriveConstants.SETTLE_TIMEOUT) {
            state = FollowState.FINISHED;
            drive.setPowers(0, 0, 0, 0);
            return;
        }

        // P control to settle at target
        // Convert world error to robot frame
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);
        double exRobot = dx * cos + dy * sin;
        double eyRobot = -dx * sin + dy * cos;

        // Use MUCH softer position gains for turn-in-place
        // For turns, we only care about heading - position correction should be minimal
        double kpSettle, kpHeadSettle, maxPosCorrection;

        if (wasTurnInPlace) {
            // NO position correction for turn-in-place settling
            // Only correct heading - position drift during turn is acceptable
            kpSettle = 0.0;          // ZERO position correction
            kpHeadSettle = 2.0;      // Slightly softer heading correction
            maxPosCorrection = 0.0;  // No position correction at all
        } else {
            // Normal gains for path following
            kpSettle = 3.0;
            kpHeadSettle = 2.5;
            maxPosCorrection = 15.0;
        }

        double vxCmd = clamp(kpSettle * exRobot, -maxPosCorrection, maxPosCorrection);
        double vyCmd = clamp(kpSettle * eyRobot, -maxPosCorrection, maxPosCorrection);
        double omegaCmd = clamp(kpHeadSettle * headingError, -Math.toRadians(90), Math.toRadians(90));

        // Slew limit
        vxCmd = slewVx.filter(vxCmd, dt);
        vyCmd = slewVy.filter(vyCmd, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        // Store for telemetry
        lastVxCmd = vxCmd;
        lastVyCmd = vyCmd;
        lastOmegaCmd = omegaCmd;
        lastLatError = eyRobot;
        lastLongError = exRobot;
        lastHeadingError = headingError;

        // Output
        double[] powers = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxCmd, vyCmd, omegaCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, powers
        );
        drive.setPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    // ==================== UTILITY METHODS ====================

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ==================== TELEMETRY GETTERS ====================

    public double getElapsed(double nowSec) {
        return nowSec - t0;
    }

    public Trajectory.State getCurrentRef(double nowSec) {
        if (traj == null) return null;
        return traj.sample(nowSec - t0);
    }

    public String getStateString() {
        return state.toString();
    }

    public double getLongError() { return lastLongError; }
    public double getLatError() { return lastLatError; }
    public double getHeadingError() { return lastHeadingError; }
    public double getVxCmd() { return lastVxCmd; }
    public double getVyCmd() { return lastVyCmd; }
    public double getOmegaCmd() { return lastOmegaCmd; }
    public boolean wasInRecovery() { return wasInRecovery; }

    public AdvancedPIDF getPidLong() { return pidLong; }
    public AdvancedPIDF getPidLat() { return pidLat; }
    public AdvancedPIDF getPidH() { return pidH; }
}