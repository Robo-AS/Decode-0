package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Advanced Trajectory Follower with:
 * - UNIFIED trajectory type detection (classified ONCE when set)
 * - Proper handling for each trajectory type (translation, turn-in-place, combined)
 * - Full self-correction capability (overshoot, undershoot, disturbances)
 * - Frenet frame for path following, world frame for turns
 * - Pure pursuit recovery when far off path
 * - Robust settling with appropriate gains per trajectory type
 */
public class TrajectoryFollower {

    // ==================== INTERFACES ====================

    @FunctionalInterface
    public interface PoseSupplier {
        Pose2d getPose();
    }

    // ==================== TRAJECTORY TYPE ====================

    /**
     * Classification of trajectory types - determined ONCE when trajectory is set.
     * This ensures consistent handling throughout tracking and settling.
     */
    public enum TrajectoryType {
        TURN_IN_PLACE,      // Pure rotation, no translation (< 2" movement)
        TRANSLATION,        // Pure translation with minimal rotation
        COMBINED            // Both significant translation and rotation
    }

    // ==================== CONFIGURATION ====================

    // Feedforward coefficients
    private double kFFVel = 1.0;
    private double kFFAccel = 0.0;
    private double kFFOmega = 1.0;
    private double kFFAlpha = 0.05;

    // Cross-coupling gains
    private double kCrossTrackToOmega = 0.0;

    // Pure pursuit
    private double lookaheadBase = DriveConstants.LOOKAHEAD_DISTANCE;
    private double lookaheadMin = DriveConstants.MIN_LOOKAHEAD;
    private double lookaheadMax = DriveConstants.MAX_LOOKAHEAD;
    private double lookaheadVelScale = 0.15;

    // Recovery thresholds
    private double purePursuitThreshold = 6.0;
    private double maxRecoverySpeed = 30.0;

    // Trajectory type detection thresholds
    private static final double MIN_TRANSLATION_DISTANCE = 2.0;  // inches
    private static final double MIN_ROTATION_ANGLE = Math.toRadians(10);  // radians

    // ==================== COMPONENTS ====================

    private final PoseSupplier poseSupplier;
    private final MecanumDrive drive;

    // PID controllers
    private final AdvancedPIDF pidLong;
    private final AdvancedPIDF pidLat;
    private final AdvancedPIDF pidH;

    // Slew rate limiters
    private final SlewRateLimiter slewVx;
    private final SlewRateLimiter slewVy;
    private final SlewRateLimiter slewOmega;

    // ==================== STATE ====================

    public Trajectory traj = null;
    private double t0 = 0.0;
    private double lastUpdateTime = 0.0;

    // Trajectory classification (set ONCE when trajectory is assigned)
    private TrajectoryType trajectoryType = TrajectoryType.TRANSLATION;

    // Cached trajectory info
    private Pose2d trajStartPose = null;
    private Pose2d trajEndPose = null;
    private double trajTotalDistance = 0.0;
    private double trajTotalRotation = 0.0;

    // End state cache
    public Trajectory.State endState = null;

    // State machine
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

        // Initialize PID controllers
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
        pidLong.setIntegralZone(3.0);
        pidLat.setIntegralLimit(5.0);
        pidLat.setIntegralZone(2.0);
        pidH.setIntegralLimit(2.0);
        pidH.setIntegralZone(Math.toRadians(15));

        // Configure output limits
        pidLong.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidLat.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidH.setOutputLimits(-DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);

        // Configure error deadbands
        pidLong.setErrorDeadband(0.1);
        pidLat.setErrorDeadband(0.1);
        pidH.setErrorDeadband(Math.toRadians(0.5));

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
     * Automatically classifies the trajectory type for proper handling.
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
            // Cache start and end states
            Trajectory.State startState = trajectory.allStates().get(0);
            endState = trajectory.allStates().get(trajectory.allStates().size() - 1);

            trajStartPose = startState.pose;
            trajEndPose = endState.pose;
            settleTarget = trajEndPose;

            // Calculate trajectory characteristics
            trajTotalDistance = Math.hypot(
                    trajEndPose.x - trajStartPose.x,
                    trajEndPose.y - trajStartPose.y
            );
            trajTotalRotation = Math.abs(normalizeAngle(trajEndPose.heading - trajStartPose.heading));

            // CLASSIFY TRAJECTORY TYPE (done ONCE, used everywhere)
            trajectoryType = classifyTrajectory(trajTotalDistance, trajTotalRotation);

            // Determine start time
            if (alignToNearest) {
                double closestT = trajectory.closestTimeTo(poseSupplier.getPose());
                t0 = nowSec - closestT;
            } else {
                t0 = nowSec;
            }
        } else {
            endState = null;
            trajStartPose = null;
            trajEndPose = null;
            trajTotalDistance = 0;
            trajTotalRotation = 0;
            trajectoryType = TrajectoryType.TRANSLATION;
            t0 = nowSec;
        }
    }

    /**
     * Classify trajectory type based on total distance and rotation.
     */
    private TrajectoryType classifyTrajectory(double distance, double rotation) {
        boolean hasTranslation = distance >= MIN_TRANSLATION_DISTANCE;
        boolean hasRotation = rotation >= MIN_ROTATION_ANGLE;

        if (!hasTranslation && hasRotation) {
            return TrajectoryType.TURN_IN_PLACE;
        } else if (hasTranslation && !hasRotation) {
            return TrajectoryType.TRANSLATION;
        } else {
            return TrajectoryType.COMBINED;
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
        double elapsed = nowSec - t0;
        return elapsed >= traj.duration() + DriveConstants.SETTLE_TIMEOUT;
    }

    public boolean isSettled() {
        return state == FollowState.FINISHED;
    }

    // ==================== MAIN UPDATE LOOP ====================

    public void update(double nowSec) {
        double dt = Math.max(0.001, nowSec - lastUpdateTime);
        lastUpdateTime = nowSec;

        if (traj == null) {
            drive.setPowers(0, 0, 0, 0);
            state = FollowState.FINISHED;
            return;
        }

        Pose2d currentPose = poseSupplier.getPose();
        double elapsed = nowSec - t0;

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
        Trajectory.State ref = traj.sample(elapsed);

        // Calculate errors in world frame
        double dx = ref.pose.x - currentPose.x;
        double dy = ref.pose.y - currentPose.y;
        double headingError = normalizeAngle(ref.pose.heading - currentPose.heading);

        double vxRobot, vyRobot, omegaCmd;

        // Handle based on trajectory type
        switch (trajectoryType) {
            case TURN_IN_PLACE:
                // TURN IN PLACE: Only heading control, no position correction
                lastLongError = dx;
                lastLatError = dy;
                lastHeadingError = headingError;

                double vComp = DriveConstants.getVoltageCompFactor();
                double omegaFF = ref.omega * kFFOmega + ref.alpha * kFFAlpha;
                omegaFF *= vComp;

                double omegaFB = pidH.update(0, -headingError, 0, dt);

                // NO position correction during turn
                vxRobot = 0.0;
                vyRobot = 0.0;
                omegaCmd = omegaFF + omegaFB;
                break;

            case TRANSLATION:
            case COMBINED:
            default:
                // TRANSLATION or COMBINED: Full position and heading control
                vxRobot = updateTranslationTracking(currentPose, ref, dx, dy, headingError, dt);
                vyRobot = lastVyCmd;  // Set by updateTranslationTracking
                omegaCmd = lastOmegaCmd;  // Set by updateTranslationTracking
                break;
        }

        // Apply limits
        vxRobot = slewVx.filter(vxRobot, dt);
        vyRobot = slewVy.filter(vyRobot, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        double vMag = Math.hypot(vxRobot, vyRobot);
        if (vMag > DriveConstants.MAX_VEL_IN_S) {
            double scale = DriveConstants.MAX_VEL_IN_S / vMag;
            vxRobot *= scale;
            vyRobot *= scale;
        }
        omegaCmd = clamp(omegaCmd, -DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);

        // Store for telemetry
        lastVxCmd = vxRobot;
        lastVyCmd = vyRobot;
        lastOmegaCmd = omegaCmd;

        // Output to drive
        double[] powers = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxRobot, vyRobot, omegaCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, powers
        );
        drive.setPowers(powers[0], powers[1], powers[2], powers[3]);

        // Check for settling
        if (elapsed >= traj.duration()) {
            state = FollowState.SETTLING;
            settleStartTime = lastUpdateTime;
        }
    }

    /**
     * Handle translation tracking (used for TRANSLATION and COMBINED types).
     * Returns vxRobot; also sets lastVyCmd and lastOmegaCmd.
     */
    private double updateTranslationTracking(Pose2d currentPose, Trajectory.State ref,
                                             double dx, double dy, double headingError, double dt) {
        // Decompose error into Frenet frame (tangent/normal)
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

        double longError = dx * tx + dy * ty;
        double latError = dx * nx + dy * ny;

        // Check for recovery mode
        if (Math.abs(latError) > purePursuitThreshold) {
            state = FollowState.RECOVERING;
            wasInRecovery = true;
            return 0;
        }

        lastLongError = longError;
        lastLatError = latError;
        lastHeadingError = headingError;

        // Feedforward
        double vComp = DriveConstants.getVoltageCompFactor();
        double vffWorld_x = ref.v * tx * kFFVel + ref.a * tx * kFFAccel;
        double vffWorld_y = ref.v * ty * kFFVel + ref.a * ty * kFFAccel;
        double omegaFF = ref.omega * kFFOmega + ref.alpha * kFFAlpha;

        vffWorld_x *= vComp;
        vffWorld_y *= vComp;
        omegaFF *= vComp;

        // Feedback
        double vLongFB = pidLong.update(0, -longError, 0, dt);
        double vLatFB = pidLat.update(0, -latError, 0, dt);
        double omegaFB = pidH.update(0, -headingError, 0, dt);
        omegaFB += kCrossTrackToOmega * latError;

        // Combine
        double vFB_world_x = vLongFB * tx + vLatFB * nx;
        double vFB_world_y = vLongFB * ty + vLatFB * ny;

        double vxWorld = vffWorld_x + vFB_world_x;
        double vyWorld = vffWorld_y + vFB_world_y;
        double omegaCmd = omegaFF + omegaFB;

        // Convert to robot frame
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);
        double vxRobot = vxWorld * cos + vyWorld * sin;
        double vyRobot = -vxWorld * sin + vyWorld * cos;

        lastVyCmd = vyRobot;
        lastOmegaCmd = omegaCmd;

        return vxRobot;
    }

    // ==================== RECOVERY STATE ====================

    private void updateRecovery(Pose2d currentPose, double elapsed, double dt) {
        // Turn-in-place trajectories don't use recovery mode
        // If disturbed during turn, just continue turning
        if (trajectoryType == TrajectoryType.TURN_IN_PLACE) {
            state = FollowState.TRACKING;
            return;
        }

        double closestT = traj.closestTimeTo(currentPose);
        Trajectory.State closestState = traj.sample(closestT);

        double lookahead = lookaheadBase + closestState.v * lookaheadVelScale;
        lookahead = clamp(lookahead, lookaheadMin, lookaheadMax);

        double lookaheadT = Math.min(closestT + lookahead / Math.max(1, closestState.v), traj.duration());
        Trajectory.State targetState = traj.sample(lookaheadT);

        double dx = targetState.pose.x - currentPose.x;
        double dy = targetState.pose.y - currentPose.y;
        double dist = Math.hypot(dx, dy);

        // Check if rejoined path
        double tx = targetState.tangent.x;
        double ty = targetState.tangent.y;
        double tMag = Math.hypot(tx, ty);
        if (tMag > 1e-6) { tx /= tMag; ty /= tMag; }
        double nx = -ty, ny = tx;
        double latError = dx * nx + dy * ny;

        if (Math.abs(latError) < purePursuitThreshold * 0.5) {
            state = FollowState.TRACKING;
            t0 = lastUpdateTime - closestT;
            return;
        }

        // Pure pursuit toward target
        double targetHeading = Math.atan2(dy, dx);
        double headingError = normalizeAngle(targetHeading - currentPose.heading);
        double endHeadingError = normalizeAngle(targetState.pose.heading - currentPose.heading);

        double speed = Math.min(maxRecoverySpeed, dist * 2.0);
        speed = Math.max(5.0, speed);

        double vxWorld = (dist > 0.1) ? (dx / dist) * speed : 0;
        double vyWorld = (dist > 0.1) ? (dy / dist) * speed : 0;

        double omegaCmd = pidH.update(0, -headingError * 0.5 - endHeadingError * 0.5, 0, dt);

        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);
        double vxRobot = vxWorld * cos + vyWorld * sin;
        double vyRobot = -vxWorld * sin + vyWorld * cos;

        vxRobot = slewVx.filter(vxRobot, dt);
        vyRobot = slewVy.filter(vyRobot, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        lastVxCmd = vxRobot;
        lastVyCmd = vyRobot;
        lastOmegaCmd = omegaCmd;
        lastLatError = latError;

        double[] powers = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxRobot, vyRobot, omegaCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, powers
        );
        drive.setPowers(powers[0], powers[1], powers[2], powers[3]);

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

        double dx = settleTarget.x - currentPose.x;
        double dy = settleTarget.y - currentPose.y;
        double posError = Math.hypot(dx, dy);
        double headingError = normalizeAngle(settleTarget.heading - currentPose.heading);

        // SAFETY: If pushed very far off target during settling, go back to recovery
        // (except for turn-in-place which doesn't care about position)
        if (trajectoryType != TrajectoryType.TURN_IN_PLACE && posError > purePursuitThreshold * 2) {
            state = FollowState.RECOVERING;
            wasInRecovery = true;
            return;
        }

        // Tolerances based on trajectory type
        double posTolerance;
        switch (trajectoryType) {
            case TURN_IN_PLACE:
                posTolerance = DriveConstants.END_POSITION_TOLERANCE * 5.0;  // Very loose for turns
                break;
            default:
                posTolerance = DriveConstants.END_POSITION_TOLERANCE;
                break;
        }

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
            settleStartTime = nowSec;
        }

        if ((nowSec - settleStartTime) > DriveConstants.SETTLE_TIMEOUT) {
            state = FollowState.FINISHED;
            drive.setPowers(0, 0, 0, 0);
            return;
        }

        // Convert to robot frame
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);
        double exRobot = dx * cos + dy * sin;
        double eyRobot = -dx * sin + dy * cos;

        // Gains based on trajectory type
        double kpSettle, kpHeadSettle, maxPosCorrection;
        switch (trajectoryType) {
            case TURN_IN_PLACE:
                // Turn: NO position correction, only heading
                kpSettle = 0.0;
                kpHeadSettle = 2.0;
                maxPosCorrection = 0.0;
                break;
            case TRANSLATION:
            case COMBINED:
            default:
                // Translation: Full correction
                kpSettle = 3.0;
                kpHeadSettle = 2.5;
                maxPosCorrection = 15.0;
                break;
        }

        double vxCmd = clamp(kpSettle * exRobot, -maxPosCorrection, maxPosCorrection);
        double vyCmd = clamp(kpSettle * eyRobot, -maxPosCorrection, maxPosCorrection);
        double omegaCmd = clamp(kpHeadSettle * headingError, -Math.toRadians(90), Math.toRadians(90));

        vxCmd = slewVx.filter(vxCmd, dt);
        vyCmd = slewVy.filter(vyCmd, dt);
        omegaCmd = slewOmega.filter(omegaCmd, dt);

        lastVxCmd = vxCmd;
        lastVyCmd = vyCmd;
        lastOmegaCmd = omegaCmd;
        lastLongError = exRobot;
        lastLatError = eyRobot;
        lastHeadingError = headingError;

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

    public double getElapsed(double nowSec) { return nowSec - t0; }

    public Trajectory.State getCurrentRef(double nowSec) {
        if (traj == null) return null;
        return traj.sample(nowSec - t0);
    }

    public String getStateString() { return state.toString(); }
    public String getTrajectoryTypeString() { return trajectoryType.toString(); }
    public TrajectoryType getTrajectoryType() { return trajectoryType; }

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