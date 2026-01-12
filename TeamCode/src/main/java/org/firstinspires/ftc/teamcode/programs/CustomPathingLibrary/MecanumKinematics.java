package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Mecanum drive kinematics utilities.
 *
 * Coordinate convention:
 * - +X: Robot forward
 * - +Y: Robot left
 * - +ω (omega): Counter-clockwise rotation
 *
 * Wheel order: [LF, RF, LB, RB] (Left-Front, Right-Front, Left-Back, Right-Back)
 *
 * Standard mecanum equations (X-configuration, rollers at 45°):
 *   LF = vx - vy - k*ω
 *   RF = vx + vy + k*ω
 *   LB = vx + vy - k*ω
 *   RB = vx - vy + k*ω
 *
 * Where k = (trackwidth + wheelbase) / 2
 */
public class MecanumKinematics {

    /**
     * Convert robot-centric velocities to wheel linear speeds.
     *
     * @param vx     Forward velocity (in/s), +X forward
     * @param vy     Strafe velocity (in/s), +Y left
     * @param omega  Angular velocity (rad/s), +CCW
     * @param track  Track width (in)
     * @param wheelbase  Wheel base (in)
     * @param out4   Output array [LF, RF, LB, RB] wheel speeds (in/s)
     */
    public static void chassisToWheels(double vx, double vy, double omega,
                                       double track, double wheelbase, double[] out4) {
        if (out4 == null || out4.length != 4) {
            throw new IllegalArgumentException("Output array must have length 4");
        }

        double k = (track + wheelbase) / 2.0;

        // Standard mecanum inverse kinematics
        out4[0] = vx - vy - k * omega; // LF
        out4[1] = vx + vy + k * omega; // RF
        out4[2] = vx + vy - k * omega; // LB
        out4[3] = vx - vy + k * omega; // RB
    }

    /**
     * Convert wheel speeds to robot-centric velocities (forward kinematics).
     *
     * @param wheelSpeeds  [LF, RF, LB, RB] wheel speeds (in/s)
     * @param track  Track width (in)
     * @param wheelbase  Wheel base (in)
     * @return  [vx, vy, omega]
     */
    public static double[] wheelsToChassis(double[] wheelSpeeds, double track, double wheelbase) {
        if (wheelSpeeds == null || wheelSpeeds.length != 4) {
            throw new IllegalArgumentException("Wheel speeds array must have length 4");
        }

        double lf = wheelSpeeds[0];
        double rf = wheelSpeeds[1];
        double lb = wheelSpeeds[2];
        double rb = wheelSpeeds[3];

        double k = (track + wheelbase) / 2.0;

        // Forward kinematics (solve the system of equations)
        double vx = (lf + rf + lb + rb) / 4.0;
        double vy = (-lf + rf + lb - rb) / 4.0;
        double omega = (-lf + rf - lb + rb) / (4.0 * k);

        return new double[]{vx, vy, omega};
    }

    /**
     * Normalize wheel speeds to motor powers [-1, 1].
     * Preserves relative ratios if any wheel exceeds max speed.
     *
     * @param wheelSpeeds  Input wheel speeds (in/s)
     * @param maxWheelSpeed  Maximum achievable wheel speed (in/s)
     * @param out4  Output motor powers [-1, 1]
     */
    public static void normalizeToPowers(double[] wheelSpeeds, double maxWheelSpeed, double[] out4) {
        if (wheelSpeeds == null || out4 == null || wheelSpeeds.length != 4 || out4.length != 4) {
            throw new IllegalArgumentException("Arrays must have length 4");
        }
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) {
            out4[0] = out4[1] = out4[2] = out4[3] = 0.0;
            return;
        }

        // Find maximum magnitude
        double maxMag = 0.0;
        for (int i = 0; i < 4; i++) {
            double mag = Math.abs(wheelSpeeds[i]);
            if (mag > maxMag) maxMag = mag;
        }

        // If all zeros, output zeros
        if (maxMag < 1e-9) {
            out4[0] = out4[1] = out4[2] = out4[3] = 0.0;
            return;
        }

        // Scale down if exceeding capability
        double scale = (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;

        // Convert to normalized power
        for (int i = 0; i < 4; i++) {
            double power = (wheelSpeeds[i] * scale) / maxWheelSpeed;

            // Apply static friction compensation if motor should be moving
            if (Math.abs(power) > DriveConstants.POWER_DEADBAND) {
                power += Math.copySign(DriveConstants.KS_POWER, power);
            } else {
                power = 0.0;
            }

            // Final clamp
            out4[i] = Math.max(-1.0, Math.min(1.0, power));
        }
    }

    /**
     * Calculate the saturation scale factor (how much we had to reduce commands).
     * Returns 1.0 if no saturation, <1.0 if wheels were saturated.
     */
    public static double computeSaturationScale(double[] wheelSpeeds, double maxWheelSpeed) {
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) return 0.0;

        double maxMag = 0.0;
        for (double w : wheelSpeeds) {
            maxMag = Math.max(maxMag, Math.abs(w));
        }

        if (maxMag < 1e-9) return 1.0;
        return (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;
    }

    /**
     * Translation-prioritized wheel power allocation.
     *
     * Guarantees translation commands are satisfied first, then allocates
     * remaining wheel authority to rotation. This prevents rotation from
     * stealing translational accuracy.
     *
     * @param vx     Forward velocity command (in/s)
     * @param vy     Strafe velocity command (in/s)
     * @param omega  Angular velocity command (rad/s)
     * @param track  Track width (in)
     * @param wheelbase  Wheel base (in)
     * @param maxWheelSpeed  Maximum wheel speed (in/s)
     * @param out4   Output motor powers [-1, 1], order [LF, RF, LB, RB]
     */
    public static void toWheelPowersPrioritized(
            double vx, double vy, double omega,
            double track, double wheelbase,
            double maxWheelSpeed, double[] out4) {

        if (out4 == null || out4.length != 4) {
            throw new IllegalArgumentException("Output array must have length 4");
        }
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) {
            out4[0] = out4[1] = out4[2] = out4[3] = 0.0;
            return;
        }

        // Sanitize inputs
        if (!Double.isFinite(vx)) vx = 0;
        if (!Double.isFinite(vy)) vy = 0;
        if (!Double.isFinite(omega)) omega = 0;

        double k = (track + wheelbase) / 2.0;

        // Step 1: Calculate wheel speeds for pure translation (no rotation)
        double[] wTrans = new double[4];
        chassisToWheels(vx, vy, 0.0, track, wheelbase, wTrans);
        double transMax = maxAbs(wTrans);

        // Step 2: Calculate wheel speeds for unit rotation (1 rad/s)
        double[] wRotUnit = new double[4];
        chassisToWheels(0.0, 0.0, 1.0, track, wheelbase, wRotUnit);
        double rotUnitMax = maxAbs(wRotUnit); // This equals k

        // Step 3: Scale translation to fit
        double transScale = 1.0;
        if (transMax > maxWheelSpeed) {
            transScale = maxWheelSpeed / transMax;
        }

        // Step 4: Calculate remaining wheel authority for rotation
        double usedCapacity = transMax * transScale;
        double remainingCapacity = maxWheelSpeed - usedCapacity;

        // Step 5: Scale rotation to fit remaining capacity
        double maxOmegaAllowed = (rotUnitMax > 1e-9) ? (remainingCapacity / rotUnitMax) : 0.0;
        double omegaScale = 1.0;
        if (Math.abs(omega) > maxOmegaAllowed && maxOmegaAllowed > 1e-9) {
            omegaScale = maxOmegaAllowed / Math.abs(omega);
        }

        // Step 6: Combine scaled translation and rotation
        double scaledVx = vx * transScale;
        double scaledVy = vy * transScale;
        double scaledOmega = omega * omegaScale;

        double[] wheelSpeeds = new double[4];
        chassisToWheels(scaledVx, scaledVy, scaledOmega, track, wheelbase, wheelSpeeds);

        // Step 7: Convert to motor powers
        for (int i = 0; i < 4; i++) {
            double power = wheelSpeeds[i] / maxWheelSpeed;

            // Apply deadband
            if (Math.abs(power) < DriveConstants.POWER_DEADBAND) {
                power = 0.0;
            } else {
                // Static friction compensation
                power += Math.copySign(DriveConstants.KS_POWER, power);
            }

            // Final clamp
            out4[i] = Math.max(-1.0, Math.min(1.0, power));
        }
    }

    /**
     * Balanced wheel power allocation.
     *
     * Scales both translation and rotation proportionally if saturation occurs.
     * Better for when both translation and rotation are important.
     *
     * @param vx     Forward velocity command (in/s)
     * @param vy     Strafe velocity command (in/s)
     * @param omega  Angular velocity command (rad/s)
     * @param track  Track width (in)
     * @param wheelbase  Wheel base (in)
     * @param maxWheelSpeed  Maximum wheel speed (in/s)
     * @param out4   Output motor powers [-1, 1]
     */
    public static void toWheelPowersBalanced(
            double vx, double vy, double omega,
            double track, double wheelbase,
            double maxWheelSpeed, double[] out4) {

        if (out4 == null || out4.length != 4) {
            throw new IllegalArgumentException("Output array must have length 4");
        }
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) {
            out4[0] = out4[1] = out4[2] = out4[3] = 0.0;
            return;
        }

        // Sanitize inputs
        if (!Double.isFinite(vx)) vx = 0;
        if (!Double.isFinite(vy)) vy = 0;
        if (!Double.isFinite(omega)) omega = 0;

        // Calculate wheel speeds
        double[] wheelSpeeds = new double[4];
        chassisToWheels(vx, vy, omega, track, wheelbase, wheelSpeeds);

        // Scale if needed
        double maxMag = maxAbs(wheelSpeeds);
        double scale = (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;

        // Convert to powers
        for (int i = 0; i < 4; i++) {
            double power = (wheelSpeeds[i] * scale) / maxWheelSpeed;

            if (Math.abs(power) < DriveConstants.POWER_DEADBAND) {
                power = 0.0;
            } else {
                power += Math.copySign(DriveConstants.KS_POWER, power);
            }

            out4[i] = Math.max(-1.0, Math.min(1.0, power));
        }
    }

    /**
     * Field-centric to robot-centric velocity conversion.
     *
     * @param vxField  Field-frame X velocity
     * @param vyField  Field-frame Y velocity
     * @param robotHeading  Robot heading in field frame (radians)
     * @return  [vxRobot, vyRobot]
     */
    public static double[] fieldToRobot(double vxField, double vyField, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double vxRobot = vxField * cos + vyField * sin;
        double vyRobot = -vxField * sin + vyField * cos;

        return new double[]{vxRobot, vyRobot};
    }

    /**
     * Robot-centric to field-centric velocity conversion.
     */
    public static double[] robotToField(double vxRobot, double vyRobot, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double vxField = vxRobot * cos - vyRobot * sin;
        double vyField = vxRobot * sin + vyRobot * cos;

        return new double[]{vxField, vyField};
    }

    // ==================== HELPER METHODS ====================

    private static double maxAbs(double[] arr) {
        double max = 0;
        for (double v : arr) {
            max = Math.max(max, Math.abs(v));
        }
        return max;
    }
}