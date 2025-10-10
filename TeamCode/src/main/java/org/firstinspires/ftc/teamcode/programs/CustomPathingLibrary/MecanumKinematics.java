package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class MecanumKinematics {

    /**
     * Map chassis twist (vx, vy, omega) to wheel linear speeds (in/s).
     */
    public static void chassisToWheels(double vx, double vy, double omega,
                                       double track, double wheelbase, double[] out4) {
        double L = (track + wheelbase) / 2.0;
        // LF, RF, LB, RB (FTC standard)
        out4[0] = vx - vy - L * omega; // LF
        out4[1] = vx + vy + L * omega; // RF
        out4[2] = vx + vy - L * omega; // LB
        out4[3] = vx - vy + L * omega; // RB
    }

    /**
     * Convert wheel linear speeds (in/s) to motor powers [-1,1] while preserving direction
     * and respecting the max wheel linear speed capability.
     */
    public static void normalizeToPowers(double[] wheelSpeeds, double maxWheelSpeed, double[] out4) {
        if (wheelSpeeds == null || out4 == null || wheelSpeeds.length != 4 || out4.length != 4) {
            throw new IllegalArgumentException("Input and output arrays must have length 4.");
        }
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) {
            for (int i = 0; i < 4; i++) out4[i] = 0.0;
            return;
        }

        // Find the largest magnitude speed
        double maxMag = 0.0;
        for (int i = 0; i < 4; i++) {
            double m = Math.abs(wheelSpeeds[i]);
            if (m > maxMag) maxMag = m;
        }
        if (maxMag < 1e-9) { // all zeros
            for (int i = 0; i < 4; i++) out4[i] = 0.0;
            return;
        }

        // If any wheel exceeds capability, scale all proportionally
        double scale = (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;

        // Convert to power in [-1,1]
        for (int i = 0; i < 4; i++) {
            double lin = wheelSpeeds[i] * scale;
            double p = lin / maxWheelSpeed;
            // clamp for safety
            if (p > 1.0) p = 1.0;
            if (p < -1.0) p = -1.0;

            //kS Static Friction compensation
            if (Math.abs(p) > 1e-6) {
                double s = Math.signum(p);
                p = s * Math.max(Math.abs(p), DriveConstants.KS_POWER);
            }

            out4[i] = p;
        }
    }

    /**
     * Returns the scaling factor that was (or would be) applied by normalizeToPowers.
     */
    public static double computeSaturationScale(double[] wheelSpeeds, double maxWheelSpeed) {
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) return 0.0;
        double maxMag = 0.0;
        for (double w : wheelSpeeds) maxMag = Math.max(maxMag, Math.abs(w));
        if (maxMag < 1e-9) return 1.0;
        return (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;
    }

    // ---------- NEW: translation-first allocator ----------

    private static double maxAbs(double[] a) {
        double m = 0;
        for (double v : a) m = Math.max(m, Math.abs(v));
        return m;
    }

    /**
     * Translation-first allocator:
     * 1) keep (vx,vy) exact if possible
     * 2) fit as much omega as remaining wheel-speed margin allows
     * 3) if translation alone exceeds max, scale translation down first, then add as much omega as possible
     * Produces motor powers in [-1, 1].
     */
    public static void toWheelPowersPrioritized(
            double vx, double vy, double omega,
            double track, double wheelbase,
            double maxWheelSpeed, double[] out4) {
        if (out4 == null || out4.length != 4)
            throw new IllegalArgumentException("out4 must have length 4.");
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) {
            out4[0] = out4[1] = out4[2] = out4[3] = 0;
            return;
        }

        // 1) Build “basis” wheel-speed vectors for pure translation and pure rotation
        double[] wT = new double[4];
        double[] wW = new double[4];

        // Pure translation
        chassisToWheels(vx, vy, 0.0, track, wheelbase, wT);
        double tMag = maxAbs(wT);

        // Pure rotation at ω = 1 rad/s (unit rotation vector)
        chassisToWheels(0.0, 0.0, 1.0, track, wheelbase, wW);
        double wMag = maxAbs(wW);

        // 2) Translation-first composition:
        //    - If translation fits, spend leftover on rotation.
        //    - If translation doesn’t fit, scale it to fit and allow 0 leftover for rotation.
        double alpha;          // translation scale
        double betaMax;        // max allowable rotation scale (rad/s) given leftover
        if (tMag <= maxWheelSpeed) {
            alpha = 1.0;
            double margin = maxWheelSpeed - tMag;                  // leftover capacity
            betaMax = (wMag > 1e-9) ? (margin / wMag) : 0.0;       // rad/s
        } else {
            alpha = maxWheelSpeed / Math.max(1e-9, tMag);          // scale translation to fit
            betaMax = 0.0;                                         // no room for rotation
        }

        // Clamp requested omega to the allowable beta band (preserves sign)
        double beta = omega;
        if (Math.abs(beta) > betaMax) beta = Math.copySign(betaMax, beta);

        // 3) Combine: wheel speeds (in/s)
        double[] w = new double[4];
        for (int i = 0; i < 4; i++) w[i] = alpha * wT[i] + beta * wW[i];

        // 4) Convert to motor powers with deadband & static-friction bump
        //    p = wheelLinearSpeed / maxWheelSpeed  (then small “ks” nudge if moving)
        double commandMag = Math.abs(vx) + Math.abs(vy) + Math.abs(omega);
        for (int i = 0; i < 4; i++) {
            double p = w[i] / maxWheelSpeed;

            // Small deadband to prevent jitter
            if (Math.abs(p) < DriveConstants.POWER_DEADBAND) {
                p = 0.0;
            } else {
                // If we intend to move, help break static friction with a tiny sign-preserving bias
                if (commandMag > 1e-6) {
                    p += Math.copySign(DriveConstants.KS_POWER, p);
                }
            }

            // Final clamp for safety
            if (p > 1.0) p = 1.0;
            if (p < -1.0) p = -1.0;
            out4[i] = p;
        }
    }
}