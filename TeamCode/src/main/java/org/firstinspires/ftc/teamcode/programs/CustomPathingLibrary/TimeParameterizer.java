package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** Jerk/accel-limited time parameterization over a CompositePath with angular caps. */
public class TimeParameterizer {

    public static class TrajSample {
        public final double t, s, v, a;     // time, arclength, vel, accel (along-track)
        public final Pose2d pose;           // x,y,heading (tangent or custom)
        public final Vector2d tangent;      // unit tangent
        public final double curvature;      // kappa
        public final double omega;          // heading rate (rad/s)
        public final double alpha;          // heading accel (rad/s^2)

        public TrajSample(double t, double s, double v, double a,
                          Pose2d pose, Vector2d tangent, double curvature,
                          double omega, double alpha) {
            this.t = t;
            this.s = s;
            this.v = v;
            this.a = a;
            this.pose = pose;
            this.tangent = tangent;
            this.curvature = curvature;
            this.omega = omega;
            this.alpha = alpha;
        }
    }

    public static List<TrajSample> parameterize(CompositePath path,
                                                TrajectoryConstraints c,
                                                double ds,
                                                HeadingProfile headingProfileOrNull, Double unwrapSeedOrNull) {

        final double Lpath = path.length();
        final double dsUse = Math.max(1e-3, ds);
        final int N = Math.max(2, (int) Math.ceil(Lpath / dsUse));

        // path samples
        double[] s     = new double[N];
        Vector2d[] pos = new Vector2d[N];
        Vector2d[] tan = new Vector2d[N];
        double[] kappa = new double[N];
        double[] theta = new double[N];

        // kinematics
        double[] v = new double[N];
        double[] a = new double[N];

        // mecanum wheel-speed feasibility helper
        final double L = 0.5 * (DriveConstants.TRACKWIDTH_IN + DriveConstants.WHEELBASE_IN);

        // geometry pass
        for (int i = 0; i < N; i++) {
            double si = ((double) i / (N - 1)) * Lpath;
            PathSample ps = path.sampleS(si);
            s[i] = si;
            pos[i] = ps.pos;
            tan[i] = ps.tangent;
            kappa[i] = ps.curvature;

            if (headingProfileOrNull != null) {
                theta[i] = headingProfileOrNull.headingAt(si);
            } else {
                theta[i] = Math.atan2(tan[i].y, tan[i].x); // tangent heading
            }

            double vmax = c.maxVel;

            if (Math.abs(kappa[i]) > 1e-9) {
                // centripetal a_lat limit
                vmax = Math.min(vmax, Math.sqrt(Math.max(0.0, c.maxCentripetal / Math.abs(kappa[i]))));
                // angular velocity feasibility (from curvature): omega = v * kappa
                vmax = Math.min(vmax, c.maxAngVel / Math.abs(kappa[i]));
                // wheel speed bound for mecanum
                vmax = Math.min(vmax, DriveConstants.MAX_WHEEL_SPEED_IN_S / (1.0 + L * Math.abs(kappa[i])));
            }

            v[i] = vmax;
            a[i] = 0.0;
        }

        if (unwrapSeedOrNull != null) {
            theta[0] = HeadingUtil.unwrapToNear(theta[0], unwrapSeedOrNull);
        } else {
            theta[0] = HeadingUtil.wrap(theta[0]);
        }
        for (int i = 1; i < N; i++) {
            theta[i] = HeadingUtil.unwrapToNear(theta[i], theta[i-1]);
        }

        // unwrap theta for derivatives
        for (int i = 1; i < N; i++) {
            double d = theta[i] - theta[i - 1];
            while (d <= -Math.PI) { theta[i] += 2.0 * Math.PI; d += 2.0 * Math.PI; }
            while (d >   Math.PI) { theta[i] -= 2.0 * Math.PI; d -= 2.0 * Math.PI; }
        }

        // theta_s and theta_ss
        double[] thS = new double[N];
        double[] thSS = new double[N];

        for (int i = 0; i < N; i++) {
            if (i == 0) {
                thS[i] = (theta[i + 1] - theta[i]) / Math.max(1e-9, s[i + 1] - s[i]);
            } else if (i == N - 1) {
                thS[i] = (theta[i] - theta[i - 1]) / Math.max(1e-9, s[i] - s[i - 1]);
            } else {
                double dl = (theta[i] - theta[i - 1]) / Math.max(1e-9, s[i] - s[i - 1]);
                double dr = (theta[i + 1] - theta[i]) / Math.max(1e-9, s[i + 1] - s[i]);
                thS[i] = 0.5 * (dl + dr);
            }
        }
        for (int i = 1; i < N - 1; i++) {
            double dl = (theta[i] - theta[i - 1]) / Math.max(1e-9, s[i] - s[i - 1]);
            double dr = (theta[i + 1] - theta[i]) / Math.max(1e-9, s[i + 1] - s[i]);
            thSS[i] = (dr - dl) / Math.max(1e-9, s[i + 1] - s[i - 1]);
        }
        thSS[0] = thSS[1];
        thSS[N - 1] = thSS[N - 2];

        // IMPORTANT: apply the heading-slope ω cap ONLY when using a custom heading profile.
        // When heading == tangent, the curvature ω cap already enforces the same thing,
        // and double-capping makes v artificially tiny → huge durations.
        if (headingProfileOrNull != null) {
            for (int i = 0; i < N; i++) {
                if (Math.abs(thS[i]) > 1e-9) {
                    v[i] = Math.min(v[i], c.maxAngVel / Math.abs(thS[i]));
                }
            }
        }

        // forward pass (accel)
        v[0] = 0.0;
        a[0] = 0.0;
        for (int i = 1; i < N; i++) {
            double ds_i = s[i] - s[i - 1];
            if (ds_i <= 0) continue;

            double vPrev = Math.max(1e-3, v[i - 1]);
            double dtEst = ds_i / vPrev;
            double aStep = c.maxJerk * dtEst;
            double aMax = Math.min(c.maxAccel, a[i - 1] + aStep);

            // angular accel cap: |theta_ss * v^2 + theta_s * a| <= maxAngAccel
            double rhs = c.maxAngAccel - Math.abs(thSS[i]) * (v[i - 1] * v[i - 1]);
            if (rhs < 0) rhs = 0;
            if (Math.abs(thS[i]) > 1e-9) {
                aMax = Math.min(aMax, rhs / Math.abs(thS[i]));
            }

            // curvature-based alpha ~ a*|kappa|
            if (Math.abs(kappa[i]) > 1e-9) {
                aMax = Math.min(aMax, c.maxAngAccel / Math.abs(kappa[i]));
            }

            double vBound = Math.sqrt(Math.max(0.0, v[i - 1] * v[i - 1] + 2.0 * aMax * ds_i));
            v[i] = Math.min(v[i], vBound);
            a[i] = aMax;
        }

        // backward pass (use positive decel magnitude)
        v[N - 1] = 0.0;
        a[N - 1] = 0.0;
        for (int i = N - 2; i >= 0; i--) {
            double ds_i = s[i + 1] - s[i];
            if (ds_i <= 0) continue;

            double vNext = Math.max(1e-3, v[i + 1]);
            double dtEst = ds_i / vNext;
            double aStep = c.maxJerk * dtEst;
            double aDecel = Math.min(c.maxDecel, a[i + 1] + aStep); // positive magnitude

            if (Math.abs(kappa[i]) > 1e-9) {
                aDecel = Math.min(aDecel, c.maxAngDecel / Math.abs(kappa[i]));
            }

            double rhs = c.maxAngAccel - Math.abs(thSS[i]) * (v[i + 1] * v[i + 1]);
            if (rhs < 0) rhs = 0;
            if (Math.abs(thS[i]) > 1e-9) {
                aDecel = Math.min(aDecel, rhs / Math.abs(thS[i]));
            }

            double vBound = Math.sqrt(Math.max(0.0, v[i + 1] * v[i + 1] + 2.0 * aDecel * ds_i));
            v[i] = Math.min(v[i], vBound);
        }

        // avoid tiny zeros which make dt huge
        for (int i = 0; i < N; i++) {
            if (v[i] < 1e-3) v[i] = 1e-3;
        }

        // recompute along-track acceleration from v profile
        a[0] = (v[1] * v[1] - v[0] * v[0]) / Math.max(1e-9, 2.0 * (s[1] - s[0]));
        for (int i = 1; i < N; i++) {
            double ds_i = Math.max(1e-9, s[i] - s[i - 1]);
            a[i] = (v[i] * v[i] - v[i - 1] * v[i - 1]) / (2.0 * ds_i);
        }

        // time integration
        List<TrajSample> out = new ArrayList<>(N);
        double t = 0.0;

        double omega0 = thS[0] * v[0];
        double alpha0 = thSS[0] * v[0] * v[0] + thS[0] * a[0];
        out.add(new TrajSample(0.0, s[0], v[0], a[0],
                new Pose2d(pos[0].x, pos[0].y, theta[0]),
                tan[0], kappa[0], omega0, alpha0));

        for (int i = 1; i < N; i++) {
            double ds_i = s[i] - s[i - 1];
            double vAvg = Math.max(1e-3, 0.5 * (v[i] + v[i - 1]));
            double dt = ds_i / vAvg;
            t += dt;

            double omega = thS[i] * v[i];
            double alpha = thSS[i] * v[i] * v[i] + thS[i] * a[i];

            out.add(new TrajSample(t, s[i], v[i], a[i],
                    new Pose2d(pos[i].x, pos[i].y, theta[i]),
                    tan[i], kappa[i], omega, alpha));
        }

        return out;
    }
}