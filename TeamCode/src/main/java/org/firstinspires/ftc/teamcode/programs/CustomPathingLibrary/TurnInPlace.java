//package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;
//
//import java.util.ArrayList;
//import java.util.List;
//
///** Pure rotation at fixed (x,y) using a trapezoidal angular profile. */
//public final class TurnInPlace {
//
//    private TurnInPlace(){}
//
//    /**
//     * Build a pure-rotation trajectory at a fixed (x,y).
//     * @param start pose to rotate from (x,y are held constant)
//     * @param targetHeadingRad absolute target heading (rad)
//     * @param c angular limits (uses maxAngVel, maxAngAccel); jerk ignored
//     * @param dt sample period (e.g., 0.02s)
//     */
//    public static Trajectory build(Pose2d start, double targetHeadingRad,
//                                   TrajectoryConstraints c, double dt) {
//
//        // compute shortest signed rotation to the target
//        double dH = wrap(targetHeadingRad - start.heading);
//        double dir = Math.signum(dH);
//        double A = Math.abs(dH);
//
//        double wMax = Math.max(1e-6, c.maxAngVel);
//        double aMax = Math.max(1e-6, c.maxAngAccel);
//        double dtClamped = Math.max(1e-3, dt);
//
//        // triangular vs trapezoidal profile
//        double triArea = (wMax * wMax) / aMax;  // total angle if accel to wMax then decel
//        boolean triangular = A <= triArea;
//
//        double ta, tc, wPeak;
//        if (triangular) {
//            ta = Math.sqrt(A / aMax);
//            tc = 0.0;
//            wPeak = aMax * ta;
//        } else {
//            ta = wMax / aMax;
//            double accelDecelArea = triArea;
//            double cruiseArea = A - accelDecelArea;
//            tc = cruiseArea / wMax;
//            wPeak = wMax;
//        }
//
//        double T = 2.0 * ta + tc;
//
//        List<Trajectory.State> out = new ArrayList<>();
//        double t = 0.0;
//        double h = start.heading;
//
//        while (t < T) {
//            // phase
//            double omega, alpha;
//            if (t < ta) {                        // accelerate
//                omega =  dir * (aMax * t);
//                alpha =  dir * aMax;
//            } else if (t < ta + tc) {            // cruise
//                omega =  dir * wPeak;
//                alpha =  0.0;
//            } else {                             // decelerate
//                double td = t - (ta + tc);
//                omega =  dir * (wPeak - aMax * td);
//                alpha = -dir * aMax;
//            }
//
//            // emit state (linear terms zero; s,v,a along-track are 0; curvature 0)
//            out.add(new Trajectory.State(
//                    t,
//                    0.0, 0.0, 0.0,
//                    new Pose2d(start.x, start.y, h),
//                    0.0,
//                    omega, alpha
//            ));
//
//            // integrate heading (Euler)
//            h = wrap(h + omega * dtClamped);
//            t += dtClamped;
//        }
//
//        // final exact endpoint
//        h = wrap(start.heading + dir * A);
//        out.add(new Trajectory.State(
//                T,
//                0.0, 0.0, 0.0,
//                new Pose2d(start.x, start.y, h),
//                0.0,
//                0.0, 0.0
//        ));
//
//        return new Trajectory(out);
//    }
//
//    private static double wrap(double a){
//        while (a <= -Math.PI) a += 2*Math.PI;
//        while (a >   Math.PI) a -= 2*Math.PI;
//        return a;
//    }
//}