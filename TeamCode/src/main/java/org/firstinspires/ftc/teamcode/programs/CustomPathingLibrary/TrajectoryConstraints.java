package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Kinematic constraints for trajectory generation.
 *
 * These constraints are used by the TimeParameterizer to create
 * velocity profiles that respect the robot's physical limits.
 *
 * All units are in inches and radians.
 */
public class TrajectoryConstraints {

    // ==================== LINEAR CONSTRAINTS ====================

    /**
     * Maximum linear velocity (inches/second).
     * The robot will never exceed this speed along the path.
     */
    public double maxVel;

    /**
     * Maximum linear acceleration (inches/second²).
     * How quickly the robot can speed up.
     */
    public double maxAccel;

    /**
     * Maximum linear deceleration (inches/second²).
     * How quickly the robot can slow down. Positive value.
     * Often can be higher than maxAccel since friction helps.
     */
    public double maxDecel;

    /**
     * Maximum jerk (inches/second³).
     * Rate of change of acceleration. Controls smoothness.
     * Higher = snappier but rougher motion.
     */
    public double maxJerk;

    /**
     * Maximum centripetal (lateral) acceleration (inches/second²).
     * Limits speed through curves: v² * curvature ≤ maxCentripetal.
     * Too high causes wheel slip on turns.
     */
    public double maxCentripetal;

    // ==================== ANGULAR CONSTRAINTS ====================

    /**
     * Maximum angular velocity (radians/second).
     * How fast the robot can rotate.
     */
    public double maxAngVel;

    /**
     * Maximum angular acceleration (radians/second²).
     * How quickly rotation can speed up.
     */
    public double maxAngAccel;

    /**
     * Maximum angular deceleration (radians/second²).
     * How quickly rotation can slow down. Positive value.
     */
    public double maxAngDecel;

    /**
     * Maximum angular jerk (radians/second³).
     * Controls smoothness of rotation changes.
     */
    public double maxAngJerk;

    // ==================== CONSTRUCTORS ====================

    /**
     * Create constraints with basic parameters.
     * Angular constraints use defaults from DriveConstants.
     */
    public TrajectoryConstraints(double maxVel, double maxAccel, double maxDecel,
                                 double maxJerk, double maxCentripetal) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxJerk = maxJerk;
        this.maxCentripetal = maxCentripetal;

        // Default angular constraints from DriveConstants
        this.maxAngVel = DriveConstants.MAX_ANG_VEL_RAD_S;
        this.maxAngAccel = DriveConstants.MAX_ANG_ACCEL_RAD_S2;
        this.maxAngDecel = DriveConstants.MAX_ANG_DECEL_RAD_S2;
        this.maxAngJerk = DriveConstants.MAX_ANG_JERK_RAD_S3;
    }

    /**
     * Create constraints with all parameters specified.
     */
    public TrajectoryConstraints(double maxVel, double maxAccel, double maxDecel,
                                 double maxJerk, double maxCentripetal,
                                 double maxAngVel, double maxAngAccel,
                                 double maxAngDecel, double maxAngJerk) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxJerk = maxJerk;
        this.maxCentripetal = maxCentripetal;
        this.maxAngVel = maxAngVel;
        this.maxAngAccel = maxAngAccel;
        this.maxAngDecel = maxAngDecel;
        this.maxAngJerk = maxAngJerk;
    }

    /**
     * Create default constraints from DriveConstants.
     */
    public static TrajectoryConstraints createDefault() {
        return new TrajectoryConstraints(
                DriveConstants.MAX_VEL_IN_S,
                DriveConstants.MAX_ACCEL_IN_S2,
                DriveConstants.MAX_DECEL_IN_S2,
                DriveConstants.MAX_JERK_IN_S3,
                DriveConstants.MAX_CENTRIPETAL
        );
    }

    /**
     * Create conservative constraints (slower but safer).
     * Good for testing or precision movements.
     */
    public static TrajectoryConstraints createConservative() {
        return new TrajectoryConstraints(
                DriveConstants.MAX_VEL_IN_S * 0.5,
                DriveConstants.MAX_ACCEL_IN_S2 * 0.5,
                DriveConstants.MAX_DECEL_IN_S2 * 0.5,
                DriveConstants.MAX_JERK_IN_S3 * 0.5,
                DriveConstants.MAX_CENTRIPETAL * 0.5,
                DriveConstants.MAX_ANG_VEL_RAD_S * 0.5,
                DriveConstants.MAX_ANG_ACCEL_RAD_S2 * 0.5,
                DriveConstants.MAX_ANG_DECEL_RAD_S2 * 0.5,
                DriveConstants.MAX_ANG_JERK_RAD_S3 * 0.5
        );
    }

    /**
     * Create aggressive constraints (faster but riskier).
     * Good for competition when you need speed.
     */
    public static TrajectoryConstraints createAggressive() {
        return new TrajectoryConstraints(
                DriveConstants.MAX_VEL_IN_S * 0.95,
                DriveConstants.MAX_ACCEL_IN_S2 * 0.9,
                DriveConstants.MAX_DECEL_IN_S2 * 0.9,
                DriveConstants.MAX_JERK_IN_S3 * 1.2,
                DriveConstants.MAX_CENTRIPETAL * 0.85,
                DriveConstants.MAX_ANG_VEL_RAD_S * 0.9,
                DriveConstants.MAX_ANG_ACCEL_RAD_S2 * 0.9,
                DriveConstants.MAX_ANG_DECEL_RAD_S2 * 0.9,
                DriveConstants.MAX_ANG_JERK_RAD_S3 * 1.2
        );
    }

    // ==================== BUILDER-STYLE SETTERS ====================

    public TrajectoryConstraints setMaxVel(double v) {
        this.maxVel = v;
        return this;
    }

    public TrajectoryConstraints setMaxAccel(double a) {
        this.maxAccel = a;
        return this;
    }

    public TrajectoryConstraints setMaxDecel(double d) {
        this.maxDecel = d;
        return this;
    }

    public TrajectoryConstraints setMaxJerk(double j) {
        this.maxJerk = j;
        return this;
    }

    public TrajectoryConstraints setMaxCentripetal(double c) {
        this.maxCentripetal = c;
        return this;
    }

    public TrajectoryConstraints setMaxAngVel(double w) {
        this.maxAngVel = w;
        return this;
    }

    public TrajectoryConstraints setMaxAngAccel(double a) {
        this.maxAngAccel = a;
        return this;
    }

    public TrajectoryConstraints setMaxAngDecel(double d) {
        this.maxAngDecel = d;
        return this;
    }

    public TrajectoryConstraints setMaxAngJerk(double j) {
        this.maxAngJerk = j;
        return this;
    }

    // ==================== SCALING ====================

    /**
     * Create a scaled copy of these constraints.
     *
     * @param factor  Scale factor (0.5 = half speed, 2.0 = double speed)
     */
    public TrajectoryConstraints scaled(double factor) {
        factor = Math.max(0.1, Math.min(2.0, factor));
        return new TrajectoryConstraints(
                maxVel * factor,
                maxAccel * factor,
                maxDecel * factor,
                maxJerk * factor,
                maxCentripetal * factor,
                maxAngVel * factor,
                maxAngAccel * factor,
                maxAngDecel * factor,
                maxAngJerk * factor
        );
    }

    /**
     * Create a copy of these constraints.
     */
    public TrajectoryConstraints copy() {
        return new TrajectoryConstraints(
                maxVel, maxAccel, maxDecel, maxJerk, maxCentripetal,
                maxAngVel, maxAngAccel, maxAngDecel, maxAngJerk
        );
    }

    // ==================== VALIDATION ====================

    /**
     * Validate and clamp constraints to reasonable values.
     */
    public TrajectoryConstraints validate() {
        maxVel = Math.max(1.0, maxVel);
        maxAccel = Math.max(1.0, maxAccel);
        maxDecel = Math.max(1.0, maxDecel);
        maxJerk = Math.max(10.0, maxJerk);
        maxCentripetal = Math.max(1.0, maxCentripetal);
        maxAngVel = Math.max(0.1, maxAngVel);
        maxAngAccel = Math.max(0.1, maxAngAccel);
        maxAngDecel = Math.max(0.1, maxAngDecel);
        maxAngJerk = Math.max(1.0, maxAngJerk);
        return this;
    }

    @Override
    public String toString() {
        return String.format(
                "TrajectoryConstraints(v=%.1f, a=%.1f, d=%.1f, j=%.1f, c=%.1f, " +
                        "ω=%.2f, α=%.2f, αd=%.2f, αj=%.2f)",
                maxVel, maxAccel, maxDecel, maxJerk, maxCentripetal,
                maxAngVel, maxAngAccel, maxAngDecel, maxAngJerk
        );
    }
}