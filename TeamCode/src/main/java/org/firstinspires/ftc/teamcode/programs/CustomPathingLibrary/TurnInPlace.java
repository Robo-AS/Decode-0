package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** Pure in-place rotation at fixed (x,y) using trapezoidal (or triangular) angular motion. */
public final class TurnInPlace {

    private TurnInPlace(){}

    /** Build a turn-to-absolute-heading trajectory at the current (x,y). */
    public static Trajectory buildToHeading(Pose2d start, double targetHeadingRad,
                                            TrajectoryConstraints c) {
        double dH = HeadingUtil.shortestDelta(start.heading, targetHeadingRad); // signed shortest
        return buildRelative(start, dH, c);
    }

    /** Build a turn-by-delta trajectory (+ = CCW) at the current (x,y). */
    public static Trajectory buildRelative(Pose2d start, double deltaHeadingRad,
                                           TrajectoryConstraints c) {

        final double A = Math.abs(deltaHeadingRad);      // total angle to sweep (rad)
        final double dir = Math.signum(deltaHeadingRad); // rotate +1 or -1

        // Limits (guard)
        final double wMax = Math.max(1e-6, c.maxAngVel);
        final double aMax = Math.max(1e-6, c.maxAngAccel);

        // Trapezoid vs triangular
        // Triangular area (accelerate to wMax then decel) equals wMax^2 / aMax
        final double triArea = (wMax * wMax) / aMax;
        final boolean triangular = (A <= triArea);

        final double ta, tc, wPeak;
        if (triangular) {
            // accel up then down, no cruise
            ta = Math.sqrt(A / aMax);
            tc = 0.0;
            wPeak = aMax * ta;
        } else {
            ta = wMax / aMax;
            final double accelDecelArea = triArea;
            final double cruiseArea = A - accelDecelArea;
            tc = cruiseArea / wMax;
            wPeak = wMax;
        }
        final double T = 2.0 * ta + tc;

        // Sample at loop rate
        final double dt = 1.0 / Math.max(10.0, DriveConstants.LOOP_HZ);
        double t = 0.0;
        double h = start.heading;

        List<Trajectory.State> states = new ArrayList<>();
        // Emit first state (omega/alpha from phase below)
        while (t < T - 1e-12) {
            double omega, alpha;

            if (t < ta) {                       // accelerate
                omega =  dir * (aMax * t);
                alpha =  dir * aMax;
            } else if (t < ta + tc) {           // cruise
                omega =  dir * wPeak;
                alpha =  0.0;
            } else {                            // decelerate
                double td = t - (ta + tc);
                omega =  dir * (wPeak - aMax * td);
                alpha = -dir * aMax;
            }

            // Integrate heading (Euler is fine at small dt)
            double hNext = HeadingUtil.wrap(h + omega * dt);

            // Tangent: pick something unit and stable; use current facing.
            Vector2d tan = new Vector2d(Math.cos(h), Math.sin(h));

            // s, v, a along the "path" are zero (no translation)
            states.add(new Trajectory.State(
                    t,
                    0.0, 0.0, 0.0,
                    new Pose2d(start.x, start.y, h),
                    tan,
                    0.0,             // curvature
                    omega,
                    alpha
            ));

            h = hNext;
            t += dt;
        }

        // Final exact endpoint
        double hEnd = HeadingUtil.wrap(start.heading + deltaHeadingRad);
        Vector2d tanEnd = new Vector2d(Math.cos(hEnd), Math.sin(hEnd));
        states.add(new Trajectory.State(
                T,
                0.0, 0.0, 0.0,
                new Pose2d(start.x, start.y, hEnd),
                tanEnd,
                0.0,
                0.0,
                0.0
        ));

        return new Trajectory(states);
    }
}