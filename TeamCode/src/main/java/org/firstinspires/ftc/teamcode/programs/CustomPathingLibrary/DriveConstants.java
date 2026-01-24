package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Central configuration for drive constants.
 *
 * TUNING GUIDE:
 * 1. Measure your robot's physical dimensions accurately
 * 2. Characterize your motors (max velocity, acceleration)
 * 3. Tune feedforward first (KF), then P, then D, then I last
 * 4. Start conservative, increase aggressively
 */
public final class DriveConstants {

    // ==================== ROBOT GEOMETRY (MEASURE ACCURATELY!) ====================

    /**
     * Distance between left and right wheel centers (inches).
     * Measure carefully - this affects turning accuracy!
     */
    public static double TRACKWIDTH_IN = 13.78;

    /**
     * Distance between front and rear wheel centers (inches).
     * For square drivetrains, this equals TRACKWIDTH.
     */
    public static double WHEELBASE_IN = 10.63;

    /**
     * Wheel radius in inches.
     * goBILDA mecanum wheels are typically 96mm (3.78") or 104mm (4.09") diameter.
     */
    public static double WHEEL_RADIUS_IN = 2.05; // 96mm diameter / 2

    // ==================== VELOCITY/ACCELERATION LIMITS ====================

    /**
     * Maximum linear velocity (inches/second).
     * Measure empirically: run robot at full power, measure distance/time.
     * Be conservative - use 80-90% of true max for reliability.
     */
    public static double MAX_VEL_IN_S = 50.0;

    /**
     * Maximum linear acceleration (inches/second²).
     * Start conservative (60-80% of max vel per second).
     */
    public static double MAX_ACCEL_IN_S2 = 50.0;

    /**
     * Maximum linear deceleration (inches/second²).
     * Can often be higher than accel since gravity/friction help.
     */
    public static double MAX_DECEL_IN_S2 = 60.0;

    /**
     * Maximum jerk (inches/second³).
     * Controls smoothness. Higher = snappier but rougher.
     * Typical range: 100-500 in/s³
     */
    public static double MAX_JERK_IN_S3 = 200.0;

    /**
     * Maximum centripetal (lateral) acceleration (inches/second²).
     * Limits speed through curves. Too high = wheel slip.
     */
    public static double MAX_CENTRIPETAL = 60.0;

    // ==================== ANGULAR LIMITS ====================

    /**
     * Maximum angular velocity (radians/second).
     * Theoretical max = MAX_VEL / (TRACKWIDTH/2), but use 70-80%.
     */
    public static double MAX_ANG_VEL_RAD_S = 4.0; // ~230 deg/s

    /**
     * Maximum angular acceleration (radians/second²).
     */
    public static double MAX_ANG_ACCEL_RAD_S2 = 4.0; // ~230 deg/s²

    /**
     * Maximum angular deceleration (radians/second²).
     */
    public static double MAX_ANG_DECEL_RAD_S2 = 5.0;

    /**
     * Maximum angular jerk (radians/second³).
     */
    public static double MAX_ANG_JERK_RAD_S3 = 15.0;

    // ==================== WHEEL SPEED ====================

    /**
     * Maximum individual wheel linear speed (inches/second).
     * This is your motor's max RPM converted to wheel speed.
     * Example: 312 RPM motor, 96mm wheel = 312/60 * π * 3.78 ≈ 62 in/s
     */
    public static double MAX_WHEEL_SPEED_IN_S = 80.709;

    // ==================== PIDF GAINS ====================
    //
    // TUNING ORDER: F → P → D → I
    //
    // KF (feedforward): Should be ~1.0 for velocity control, ~0 for position
    // KP (proportional): Start at 1-3, increase until oscillation, back off 20%
    // KD (derivative): Start at 0.1*KP, increase to dampen oscillation
    // KI (integral): Usually very small (0.001-0.01), only if steady-state error
    //

    // Longitudinal (along path) PIDF
    public static double KP_X = 4.0;
    public static double KI_X = 0.01;
    public static double KD_X = 0.3;
    public static double KF_X = 0.0; // Feedforward handled separately

    // Lateral (cross-track) PIDF
    public static double KP_Y = 5.0;  // Usually higher than X for quick correction
    public static double KI_Y = 0.01;
    public static double KD_Y = 0.4;
    public static double KF_Y = 0.0;

    // Heading PIDF
    public static double KP_H = 3.0;
    public static double KI_H = 0.005;
    public static double KD_H = 0.2;
    public static double KF_H = 0.0;

    // ==================== FEEDFORWARD COEFFICIENTS ====================

    /**
     * Static friction compensation (minimum power to overcome friction).
     * Measure by slowly increasing power until robot barely moves.
     */
    public static double KS_POWER = 0.04;

    /**
     * Velocity feedforward coefficient.
     * Power = kV * velocity. Measure by running at known velocity.
     * kV ≈ 1 / MAX_WHEEL_SPEED_IN_S for normalized output.
     */
    public static double KV = 1.0 / MAX_WHEEL_SPEED_IN_S;

    /**
     * Acceleration feedforward coefficient.
     * Compensates for inertia. Usually small (0.0001-0.001).
     */
    public static double KA = 0.0002;

    // ==================== CONTROL LOOP SETTINGS ====================

    /**
     * Control loop frequency (Hz).
     * FTC typically runs at 50-100 Hz depending on code complexity.
     */
    public static double LOOP_HZ = 50.0;

    /**
     * Derivative filter cutoff frequency (Hz).
     * Lower = more smoothing, less noise, more phase lag.
     * Typical: 10-30 Hz
     */
    public static double D_CUTOFF_HZ = 15.0;

    /**
     * Power deadband - powers below this are zeroed to prevent motor whine.
     */
    public static double POWER_DEADBAND = 0.02;

    // ==================== SLEW RATE LIMITS ====================
    // These limit how fast commands can change (jerk limiting at output stage)

    public static double MAX_DVX_IN_S2 = 200.0;  // Max change in vx per second
    public static double MAX_DVY_IN_S2 = 200.0;  // Max change in vy per second
    public static double MAX_DW_RAD_S2 = 15.0;   // Max change in omega per second

    // ==================== VOLTAGE COMPENSATION ====================

    public static final boolean ENABLE_VOLTAGE_COMP = true;

    /**
     * Nominal battery voltage where gains were tuned.
     * Fresh FTC battery is ~13.0-13.5V.
     */
    public static final double NOMINAL_BATTERY_VOLT = 13.0;

    /**
     * Minimum voltage for scaling (don't scale too aggressively on dead battery).
     */
    public static final double MIN_BATTERY_VOLT_FOR_SCALING = 11.0;

    /**
     * Maximum voltage compensation factor.
     * Prevents runaway scaling on very low battery.
     */
    public static final double MAX_VOLTAGE_COMP_FACTOR = 1.3;

    // ==================== PATH FOLLOWING TOLERANCES ====================

    /**
     * Position tolerance for "finished" detection (inches).
     */
    public static double END_POSITION_TOLERANCE = 0.5;

    /**
     * Heading tolerance for "finished" detection (radians).
     */
    public static double END_HEADING_TOLERANCE = Math.toRadians(2.0);

    /**
     * Velocity tolerance for "settled" detection (inches/second).
     */
    public static double SETTLE_VELOCITY_TOLERANCE = 1.0;

    /**
     * Time to hold within tolerance before declaring settled (seconds).
     */
    public static double SETTLE_TIME = 0.15;

    /**
     * Maximum time to wait for settling before giving up (seconds).
     */
    public static double SETTLE_TIMEOUT = 1.5;

    // ==================== PURE PURSUIT SETTINGS ====================

    /**
     * Lookahead distance for pure pursuit (inches).
     * Larger = smoother but less accurate, smaller = tighter but can oscillate.
     */
    public static double LOOKAHEAD_DISTANCE = 8.0;

    /**
     * Minimum lookahead (prevents divide-by-zero and excessive correction).
     */
    public static double MIN_LOOKAHEAD = 4.0;

    /**
     * Maximum lookahead (prevents looking too far ahead on tight paths).
     */
    public static double MAX_LOOKAHEAD = 16.0;

    // ==================== UTILITY METHODS ====================

    /**
     * Returns voltage compensation factor for feedforward scaling.
     * At lower voltage, we need more power for the same speed.
     */
    public static double getVoltageCompFactor() {
        if (!ENABLE_VOLTAGE_COMP || MecanumDrive.battery == null) {
            return 1.0;
        }

        double v = MecanumDrive.battery.getVoltage();
        if (!Double.isFinite(v) || v < 0.1) {
            return 1.0;
        }

        double vClamped = Math.max(MIN_BATTERY_VOLT_FOR_SCALING, v);
        double factor = NOMINAL_BATTERY_VOLT / vClamped;

        // Clamp factor to prevent runaway
        return Math.min(factor, MAX_VOLTAGE_COMP_FACTOR);
    }

    /**
     * Calculate theoretical max angular velocity based on geometry.
     */
    public static double getTheoreticalMaxAngVel() {
        return MAX_WHEEL_SPEED_IN_S / ((TRACKWIDTH_IN + WHEELBASE_IN) / 2.0);
    }

    /**
     * Get the "k" factor for mecanum kinematics.
     */
    public static double getMecanumK() {
        return (TRACKWIDTH_IN + WHEELBASE_IN) / 2.0;
    }

    private DriveConstants() {} // No instantiation
}