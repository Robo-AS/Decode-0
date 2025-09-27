package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** Jerk-limited S-curve time parameterization over a CompositePath. */
public class TimeParameterizer {

    public static class TrajSample {
        public final double t, s, v, a;     // time, arclength, vel, accel
        public final Pose2d pose;           // x,y,heading (heading from tangent)
        public final Vector2d tangent;
        public final double curvature;
        public final double omega;
        public final double alpha;


        public TrajSample(double t, double s, double v, double a, Pose2d pose, Vector2d tangent, double curvature, double omega, double alpha) {
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

    /**
     * Build a time-parameterized trajectory sampling the path at ds, enforcing constraints.
     */
    public static List<TrajSample> parameterize(CompositePath path, TrajectoryConstraints c, double ds, HeadingProfile headingProfileOrNull) {
        int N = (int) Math.max(2, Math.ceil(path.length() / ds));
        final double totalS = path.length();
        double[] s = new double[N];
        double[] v = new double[N];
        double[] a = new double[N];
        double[] k = new double[N];
        Vector2d[] tan = new Vector2d[N];
        Vector2d[] pos = new Vector2d[N];
        double[] theta = new double[N];

        double L =(DriveConstants.TRACKWIDTH_IN+DriveConstants.WHEELBASE_IN)*0.5;

        // initial pass: curvature-based vmax
        for (int i = 0; i < N; i++) {
            double si = (double) i / (N - 1) * path.length();
            PathSample ps = path.sampleS(si);
            s[i] = si;
            k[i] = ps.curvature;
            tan[i] = ps.tangent;
            pos[i] = ps.pos;
            double vmax = c.maxVel;
            if (headingProfileOrNull != null) {
                theta[i] = headingProfileOrNull.headingAt(si);
            } else {
                theta[i] = Math.atan2(ps.tangent.y, ps.tangent.x); //tangent heading
            }

            // 1) centripetal a_lat cap: v <= sqrt(a_lat_max / |k|)
            if (Math.abs(k[i]) > 1e-9) {
                double vCentr = Math.sqrt(Math.max(0.0, c.maxCentripetal / Math.abs(k[i])));
                vmax = Math.min(vmax, vCentr);

                // 2) angular velocity feasibility: omega = v*kappa <= maxAngVel
                double vOmega = c.maxAngVel / Math.abs(k[i]);
                vmax = Math.min(vmax, vOmega);

                // 3) wheel speed feasibility for mecanum (vx=v, vy≈0, omega=v*kappa)
                // |wheel| ≈ | v ± L*omega | = v * |1 ± L*kappa|  → v <= Vw_max / (1 + L*|k|)
                double vWheel = DriveConstants.MAX_WHEEL_SPEED_IN_S / (1.0 + L*Math.abs(k[i]));
                vmax = Math.min(vmax, vWheel);
            }
            v[i] = vmax;
            a[i] = 0.0;
        }

        //unwrap theta to avoid 2pi jumps
        for (int i = 1; i < N; i++) {
            double d = theta[i] - theta[i - 1];
            while (d <= -Math.PI) {
                theta[i] += 2 * Math.PI;
                d += 2 * Math.PI;
            }
            while (d > Math.PI) {
                theta[i] -= 2 * Math.PI;
                d -= 2 * Math.PI;
            }
        }

        //numerical w.r.ts: theta_s, theta_ss
        double[] thS = new double[N];
        double[] thSS = new double[N];
        for (int i = 0; i < N; i++) {
            if (i == 0) {
                double ds1 = Math.max(1e-9, s[i + 1] - s[i]);
                thS[i] = (theta[i + 1] - theta[i]) / ds1;
            } else if (i == N - 1) {
                double ds1 = Math.max(1e-9, s[i] - s[i - 1]);
                thS[i] = (theta[i] - theta[i - 1]) / ds1;
            } else {
                double dsL = Math.max(1e-9, s[i] - s[i - 1]);
                double dsR = Math.max(1e-9, s[i + 1] - s[i]);
                double dL = (theta[i] - theta[i - 1]) / dsL;
                double dR = (theta[i + 1] - theta[i]) / dsR;
                thS[i] = 0.5 * (dL + dR);
            }
        }

        // 2) curvature-limited vmax (linear) + ANGULAR ω limit: |theta_s|*v ≤ maxAngVel
        for (int i = 0; i < N; i++) {
            double vmaxCurve = (Math.abs(k[i]) < 1e-9)
                    ? c.maxVel
                    : Math.min(c.maxVel, Math.sqrt(Math.max(0.0, c.maxCentripetal / Math.abs(k[i]))));

            double vmaxAng = (Math.abs(thS[i]) < 1e-9)
                    ? c.maxVel
                    : (c.maxAngVel / Math.abs(thS[i])); // v ≤ ωmax / |θ_s|

            v[i] = Math.min(vmaxCurve, vmaxAng);
            a[i] = 0.0;
        }

        // 3) forward pass (jerk-limited linear accel) with angular α constraint merged
        v[0] = 0.0;
        a[0] = 0.0;


        for (int i = 1; i < N; i++) {

            double ds_i = s[i] - s[i - 1];
            if (ds_i < 1e-9) continue;

            double vPrev = Math.max(1e-6, v[i - 1]);
            double dt_est = ds_i / vPrev;
            double aMaxStep = c.maxJerk * dt_est;

            // linear accel candidate
            double aCand = Math.min(c.maxAccel, a[i - 1] + aMaxStep);

            // Approximate alpha = a*kappa (ignore curvature slope term for robustness)
            double alphaEst = a[i] * k[i];
            if (Math.abs(alphaEst) > c.maxAngAccel) {
                // reduce a[i] so that |alpha| == maxAngAccel
                double aCap = c.maxAngAccel / Math.max(1e-9, Math.abs(k[i]));
                a[i] = Math.copySign(Math.min(Math.abs(a[i]), aCap), a[i]);
                // re-compute feasible v with reduced a
                double vCand = Math.sqrt(Math.max(0.0, v[i-1]*v[i-1] + 2.0 * a[i] * (s[i]-s[i-1])));
                v[i] = Math.min(v[i], vCand);
            }

            // angular accel constraint: |alpha| = |theta_ss * v^2 + theta_s * a| ≤ maxAngAccel
            // Solve for a: |theta_s| * a ≤ maxAngAccel - |theta_ss| * v^2
            double rhs = c.maxAngAccel - Math.abs(thSS[i]) * (v[i - 1] * v[i - 1]);
            if (rhs < 0) rhs = 0;
            if (Math.abs(thS[i]) > 1e-9) {
                double aMaxFromAng = rhs / Math.abs(thS[i]);
                aCand = Math.min(aCand, aMaxFromAng);
            }

            // kinematics v^2 = v0^2 + 2*a*ds
            double vCand = Math.sqrt(Math.max(0.0, v[i - 1] * v[i - 1] + 2.0 * aCand * ds_i));
            v[i] = Math.min(v[i], vCand);
            a[i] = aCand;
        }

        // 4) backward pass (jerk-limited linear decel) with angular α constraint
        v[N - 1] = 0.0;
        a[N - 1] = 0.0;
        for (int i = N - 2; i >= 0; i--) {
            double ds_i = s[i + 1] - s[i];
            if (ds_i < 1e-9) continue;

            double vNext = Math.max(1e-6, v[i + 1]);
            double dt_est = ds_i / vNext;
            double aMaxStep = c.maxJerk * dt_est;

            // decel candidate (negative)
            double aCand = Math.max(-c.maxDecel, a[i + 1] - aMaxStep);

            // After computing v[i] and a[i] in the backward pass
            double alphaEst = a[i] * k[i]; // a[i] will be negative on decel sections
            double alphaCap = c.maxAngDecel;
            if (Math.abs(alphaEst) > alphaCap) {
                double aCap = alphaCap / Math.max(1e-9, Math.abs(k[i]));
                // on decel, a[i] is negative; clamp toward -aCap
                a[i] = Math.max(-aCap, a[i]);
                double vCand = Math.sqrt(Math.max(0.0, v[i+1]*v[i+1] + 2.0 * a[i] * (s[i] - s[i+1])));
                v[i] = Math.min(v[i], vCand);
            }


            double rhs = c.maxAngAccel - Math.abs(thSS[i]) * (v[i + 1] * v[i + 1]);
            if (rhs < 0) rhs = 0;
            if (Math.abs(thS[i]) > 1e-9) {
                double aMaxFromAng = rhs / Math.abs(thS[i]);
                // aCand is negative, so limit magnitude accordingly
                if (aCand < -aMaxFromAng) aCand = -aMaxFromAng;
            }

            double vCand = Math.sqrt(Math.max(0.0, v[i + 1] * v[i + 1] + 2.0 * aCand * ds_i));
            v[i] = Math.min(v[i], vCand);
            a[i] = aCand;
        }

        // 5) integrate time, compute omega/alpha feedforward
        List<TrajSample> out = new ArrayList<>(N);
        double t = 0.0;
        double omega = thS[0] * v[0];
        double alpha = thSS[0] * v[0] * v[0] + thS[0] * a[0];
        Pose2d pose0 = new Pose2d(pos[0].x, pos[0].y, theta[0]);
        out.add(new TrajSample(t, s[0], v[0], a[0], pose0, tan[0], k[0], omega, alpha));

        for (int i = 1; i < N; i++) {
            double ds_i = s[i] - s[i - 1];
            double vmid = Math.max(1e-6, 0.5 * (v[i] + v[i - 1]));
            double dt = ds_i / vmid;
            t += dt;

            omega = thS[i] * v[i];
            alpha = thSS[i] * v[i] * v[i] + thS[i] * a[i];
            Pose2d pose = new Pose2d(pos[i].x, pos[i].y, theta[i]);
            out.add(new TrajSample(t, s[i], v[i], a[i], pose, tan[i], k[i], omega, alpha));
        }
        return out;
    }


    private static TrajSample toSample(int i, double t, double[] s, double[] v, double[] a,
                                       Vector2d[] pos, Vector2d[] tan, double[] k, double omega, double alpha) {
        double heading = Math.atan2(tan[i].y, tan[i].x);
        Pose2d pose = new Pose2d(pos[i].x, pos[i].y, heading);
        return new TrajSample(t, s[i], v[i], a[i], pose, tan[i], k[i], omega, alpha);
    }
}
