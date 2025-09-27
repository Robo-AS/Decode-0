package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.Arrays;

public class MecanumKinematics {
    public static void chassisToWheels(double vx,double vy,double omega,
                                       double track,double wheelbase, double[] out4){
        double L = (track + wheelbase)/2.0;
        double fl =  vx - vy - L*omega;
        double fr =  vx + vy + L*omega;
        double bl =  vx + vy - L*omega;
        double br =  vx - vy + L*omega;
        out4[0]=fl; out4[1]=fr; out4[2]=bl; out4[3]=br;
    }
    public static void normalizeToPowers(double[] wheelSpeeds, double maxWheelSpeed, double[] out4) {
        if (wheelSpeeds.length != 4 || out4.length != 4) {
            throw new IllegalArgumentException("Input and output arrays must have length 4.");
        }

        // Guard against invalid max speed
        if (maxWheelSpeed < 1e-6) {
            Arrays.fill(out4, 0.0);
            return;
        }

        // Find maximum absolute wheel speed
        double maxMag = 0.0;
        for (int i = 0; i < 4; i++) {
            maxMag = Math.max(maxMag, Math.abs(wheelSpeeds[i]));
        }

        // Scaling factor if any wheel speed exceeds max
        double scale = (maxMag > maxWheelSpeed) ? (maxWheelSpeed / maxMag) : 1.0;

        // Normalize into [-1,1]
        for (int i = 0; i < 4; i++) {
            double raw = (wheelSpeeds[i] * scale) / maxWheelSpeed;
            // Clamp to be safe against floating-point creep
            out4[i] = Math.max(-1.0, Math.min(1.0, raw));
        }
    }
}
