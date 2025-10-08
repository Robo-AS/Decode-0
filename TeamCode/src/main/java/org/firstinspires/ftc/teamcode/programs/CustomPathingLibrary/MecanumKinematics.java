package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class MecanumKinematics {

    /** Map chassis twist (vx, vy, omega) to wheel linear speeds (in/s). */
    public static void chassisToWheels(double vx,double vy,double omega,
                                       double track,double wheelbase, double[] out4){
        double L = (track + wheelbase)/2.0;
        // LF, RF, LB, RB (FTC standard)
        out4[0] =  vx - vy - L*omega; // LF
        out4[1] =  vx + vy + L*omega; // RF
        out4[2] =  vx + vy - L*omega; // LB
        out4[3] =  vx - vy + L*omega; // RB
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
            for (int i=0;i<4;i++) out4[i]=0.0;
            return;
        }

        // Find the largest magnitude speed
        double maxMag = 0.0;
        for (int i = 0; i < 4; i++) {
            double m = Math.abs(wheelSpeeds[i]);
            if (m > maxMag) maxMag = m;
        }
        if (maxMag < 1e-9) { // all zeros
            for (int i=0;i<4;i++) out4[i]=0.0;
            return;
        }

        // If any wheel exceeds capability, scale all proportionally
        double scale = (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;

        // Convert to power in [-1,1]
        for (int i = 0; i < 4; i++) {
            double lin = wheelSpeeds[i] * scale;
            double p = lin / maxWheelSpeed;
            // clamp for safety
            if (p >  1.0) p =  1.0;
            if (p < -1.0) p = -1.0;
            out4[i] = p;
        }
    }

    /** Returns the scaling factor that was (or would be) applied by normalizeToPowers. */
    public static double computeSaturationScale(double[] wheelSpeeds, double maxWheelSpeed){
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 1e-6) return 0.0;
        double maxMag = 0.0;
        for (double w : wheelSpeeds) maxMag = Math.max(maxMag, Math.abs(w));
        if (maxMag < 1e-9) return 1.0;
        return (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;
    }
}