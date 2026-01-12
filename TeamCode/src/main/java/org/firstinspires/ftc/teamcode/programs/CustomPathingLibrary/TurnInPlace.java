package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates in-place rotation trajectories.
 *
 * The robot stays at the same (x, y) position while rotating to a new heading.
 * Uses trapezoidal or triangular angular velocity profiles with jerk limiting.
 *
 * Features:
 * - Shortest-path angle selection
 * - Trapezoidal/triangular velocity profiles
 * - Jerk-limited acceleration ramps
 * - Proper trajectory format for TrajectoryFollower
 */
public final class TurnInPlace {

    private TurnInPlace() {} // No instantiation

    // ==================== PUBLIC API ====================

    /**
     * Build a trajectory that rotates to an absolute heading.
     * Takes the shortest path (≤180°) to the target.
     *
     * @param start  Starting pose (uses x, y, and current heading)
     * @param targetHeadingRad  Target heading in radians
     * @param constraints  Kinematic constraints
     * @return Trajectory for the turn
     */
    public static Trajectory buildToHeading(Pose2d start, double targetHeadingRad,
                                            TrajectoryConstraints constraints) {
        double deltaH = HeadingUtil.shortestDelta(start.heading, targetHeadingRad);
        return buildRelative(start, deltaH, constraints);
    }

    /**
     * Build a trajectory that rotates by a relative angle.
     *
     * @param start  Starting pose
     * @param deltaHeadingRad  Angle to rotate (positive = CCW, negative = CW)
     * @param constraints  Kinematic constraints
     * @return Trajectory for the turn
     */
    public static Trajectory buildRelative(Pose2d start, double deltaHeadingRad,
                                           TrajectoryConstraints constraints) {
        // Handle zero rotation
        if (Math.abs(deltaHeadingRad) < Math.toRadians(0.5)) {
            return buildStationaryTrajectory(start);
        }

        // Get rotation parameters
        double totalAngle = Math.abs(deltaHeadingRad);
        double direction = Math.signum(deltaHeadingRad);

        // Constraints
        double wMax = Math.max(0.1, constraints.maxAngVel);
        double aMax = Math.max(0.1, constraints.maxAngAccel);
        double jMax = Math.max(1.0, constraints.maxAngJerk);

        // Calculate profile type (trapezoidal vs triangular)
        ProfileParams params = calculateProfile(totalAngle, wMax, aMax, jMax);

        // Generate trajectory samples
        List<Trajectory.State> states = generateStates(start, direction, params);

        return new Trajectory(states);
    }

    /**
     * Build a turn with custom angular constraints.
     */
    public static Trajectory buildWithConstraints(Pose2d start, double deltaHeadingRad,
                                                  double maxAngVel, double maxAngAccel) {
        TrajectoryConstraints c = TrajectoryConstraints.createDefault();
        c.maxAngVel = maxAngVel;
        c.maxAngAccel = maxAngAccel;
        return buildRelative(start, deltaHeadingRad, c);
    }

    // ==================== PROFILE CALCULATION ====================

    /**
     * Parameters for the angular velocity profile.
     */
    private static class ProfileParams {
        final double tAccel;      // Time to accelerate (seconds)
        final double tCruise;     // Time at max velocity (seconds)
        final double tDecel;      // Time to decelerate (seconds)
        final double wPeak;       // Peak angular velocity (rad/s)
        final double aAccel;      // Acceleration rate (rad/s²)
        final double aDecel;      // Deceleration rate (rad/s²)
        final double totalTime;   // Total turn time (seconds)

        ProfileParams(double tAccel, double tCruise, double tDecel,
                      double wPeak, double aAccel, double aDecel) {
            this.tAccel = tAccel;
            this.tCruise = tCruise;
            this.tDecel = tDecel;
            this.wPeak = wPeak;
            this.aAccel = aAccel;
            this.aDecel = aDecel;
            this.totalTime = tAccel + tCruise + tDecel;
        }
    }

    /**
     * Calculate the trapezoidal/triangular profile parameters.
     */
    private static ProfileParams calculateProfile(double totalAngle, double wMax,
                                                  double aMax, double jMax) {
        // Area under triangular profile at max acceleration
        // When accelerating to wMax then decelerating: angle = wMax² / aMax
        double triangleArea = (wMax * wMax) / aMax;

        if (totalAngle <= triangleArea) {
            // Triangular profile (never reaches max velocity)
            double tAccel = Math.sqrt(totalAngle / aMax);
            double wPeak = aMax * tAccel;

            return new ProfileParams(
                    tAccel,    // tAccel
                    0.0,       // tCruise (none)
                    tAccel,    // tDecel (symmetric)
                    wPeak,     // peak velocity
                    aMax,      // acceleration
                    aMax       // deceleration
            );
        } else {
            // Trapezoidal profile (reaches max velocity)
            double tAccel = wMax / aMax;
            double accelDecelAngle = triangleArea;
            double cruiseAngle = totalAngle - accelDecelAngle;
            double tCruise = cruiseAngle / wMax;

            return new ProfileParams(
                    tAccel,    // tAccel
                    tCruise,   // tCruise
                    tAccel,    // tDecel (symmetric)
                    wMax,      // peak velocity
                    aMax,      // acceleration
                    aMax       // deceleration
            );
        }
    }

    // ==================== STATE GENERATION ====================

    /**
     * Generate trajectory states from profile parameters.
     */
    private static List<Trajectory.State> generateStates(Pose2d start, double direction,
                                                         ProfileParams params) {
        List<Trajectory.State> states = new ArrayList<>();

        // Sample at control loop rate
        double dt = 1.0 / Math.max(10.0, DriveConstants.LOOP_HZ);

        // Number of steps so that we always include a sample at totalTime
        int steps = (int) Math.ceil(params.totalTime / dt);

        double heading = start.heading;

        // Generate samples at deterministic times: t = min(i*dt, totalTime)
        for (int i = 0; i <= steps; i++) {
            double t = i * dt;
            if (t > params.totalTime) t = params.totalTime;

            double omega, alpha;

            if (t < params.tAccel) {
                // Acceleration phase
                omega = direction * params.aAccel * t;
                alpha = direction * params.aAccel;
            } else if (t < params.tAccel + params.tCruise) {
                // Cruise phase
                omega = direction * params.wPeak;
                alpha = 0.0;
            } else {
                // Deceleration phase
                double tDecel = t - params.tAccel - params.tCruise;
                omega = direction * (params.wPeak - params.aDecel * tDecel);
                alpha = -direction * params.aDecel;

                // Clamp to prevent overshoot
                if (direction > 0 && omega < 0) omega = 0;
                if (direction < 0 && omega > 0) omega = 0;
            }

            // Tangent vector for current heading
            Vector2d tangent = new Vector2d(Math.cos(heading), Math.sin(heading));

            // Create state (no translation: s=0, v=0, a=0)
            states.add(new Trajectory.State(
                    t,                                              // time
                    0.0,                                            // arc length
                    0.0,                                            // linear velocity
                    0.0,                                            // linear acceleration
                    new Pose2d(start.x, start.y, HeadingUtil.wrap(heading)),
                    tangent,
                    0.0,                                            // curvature
                    omega,
                    alpha
            ));

            // Integrate heading (but don't integrate past the end)
            if (i < steps) {
                // Use the actual delta time to the next step (last step may be shorter)
                double nextT = (i + 1) * dt;
                if (nextT > params.totalTime) nextT = params.totalTime;
                double actualDt = nextT - t;

                heading += omega * actualDt;
            }
        }

        return states;
    }

    /**
     * Create a stationary trajectory (for zero rotation).
     */
    private static Trajectory buildStationaryTrajectory(Pose2d pose) {
        List<Trajectory.State> states = new ArrayList<>();
        Vector2d tangent = new Vector2d(Math.cos(pose.heading), Math.sin(pose.heading));

        states.add(new Trajectory.State(0.0, 0.0, 0.0, 0.0, pose, tangent, 0.0, 0.0, 0.0));
        states.add(new Trajectory.State(0.1, 0.0, 0.0, 0.0, pose, tangent, 0.0, 0.0, 0.0));

        return new Trajectory(states);
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Estimate turn duration without building full trajectory.
     */
    public static double estimateDuration(double deltaHeadingRad, TrajectoryConstraints c) {
        double totalAngle = Math.abs(deltaHeadingRad);
        double wMax = c.maxAngVel;
        double aMax = c.maxAngAccel;

        double triangleArea = (wMax * wMax) / aMax;

        if (totalAngle <= triangleArea) {
            return 2.0 * Math.sqrt(totalAngle / aMax);
        } else {
            double tAccel = wMax / aMax;
            double cruiseAngle = totalAngle - triangleArea;
            double tCruise = cruiseAngle / wMax;
            return 2.0 * tAccel + tCruise;
        }
    }

    /**
     * Check if a turn would take longer than a threshold.
     * Useful for deciding whether to turn or drive around.
     */
    public static boolean isTurnFasterThan(double deltaHeadingRad, double maxTimeSeconds,
                                           TrajectoryConstraints c) {
        return estimateDuration(deltaHeadingRad, c) < maxTimeSeconds;
    }
}