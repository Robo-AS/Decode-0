package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Time parameterization for paths with:
 * - Proper forward/backward pass velocity profiling
 * - Jerk limiting for smooth acceleration changes
 * - Angular velocity/acceleration constraints
 * - Curvature-based velocity limits
 * - Mecanum wheel speed feasibility
 * - Numerical stability improvements
 */
public class TimeParameterizer {

    /**
     * A sample point along the trajectory with full kinematic information.
     */
    public static class TrajSample {
        public final double t;          // Time (seconds)
        public final double s;          // Arc length (inches)
        public final double v;          // Linear velocity along path (in/s)
        public final double a;          // Linear acceleration along path (in/s²)
        public final Pose2d pose;       // Position and heading
        public final Vector2d tangent;  // Unit tangent vector
        public final double curvature;  // Path curvature (1/in)
        public final double omega;      // Angular velocity (rad/s)
        public final double alpha;      // Angular acceleration (rad/s²)

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

    /**
     * Parameterize a composite path into time-based trajectory samples.
     *
     * @param path                  The spatial path to parameterize
     * @param c                     Kinematic constraints
     * @param ds                    Arc-length sampling resolution (inches)
     * @param headingProfileOrNull  Custom heading profile, or null for tangent-following
     * @param unwrapSeedOrNull      Starting heading for unwrapping, or null
     * @return List of trajectory samples
     */
    public static List<TrajSample> parameterize(CompositePath path,
                                                TrajectoryConstraints c,
                                                double ds,
                                                HeadingProfile headingProfileOrNull,
                                                Double unwrapSeedOrNull) {

        final double pathLength = path.length();
        if (pathLength < 1e-6) {
            // Degenerate path - return single stationary point
            return createStationaryTrajectory(path, headingProfileOrNull);
        }

        // Determine number of samples
        ds = Math.max(0.1, Math.min(2.0, ds));
        final int N = Math.max(10, (int) Math.ceil(pathLength / ds) + 1);

        // Arrays for path geometry
        double[] s = new double[N];
        Vector2d[] pos = new Vector2d[N];
        Vector2d[] tan = new Vector2d[N];
        double[] kappa = new double[N];  // Curvature
        double[] theta = new double[N];  // Heading

        // Arrays for velocity profile
        double[] vMax = new double[N];   // Maximum allowed velocity at each point
        double[] v = new double[N];      // Actual velocity profile
        double[] a = new double[N];      // Acceleration profile

        // Mecanum geometry constant
        final double mecanumK = (DriveConstants.TRACKWIDTH_IN + DriveConstants.WHEELBASE_IN) / 2.0;

        // ==================== PASS 1: GEOMETRY & LOCAL VELOCITY LIMITS ====================

        for (int i = 0; i < N; i++) {
            double si = ((double) i / (N - 1)) * pathLength;
            PathSample ps = path.sampleS(si);

            s[i] = si;
            pos[i] = ps.pos;
            tan[i] = ps.tangent;
            kappa[i] = ps.curvature;

            // Get heading
            if (headingProfileOrNull != null) {
                theta[i] = headingProfileOrNull.headingAt(si);
            } else {
                theta[i] = Math.atan2(ps.tangent.y, ps.tangent.x);
            }

            // Calculate local velocity limit
            double vLimit = c.maxVel;

            // Centripetal acceleration limit: v² * |κ| ≤ a_lat
            if (Math.abs(kappa[i]) > 1e-9) {
                double vCentripetal = Math.sqrt(c.maxCentripetal / Math.abs(kappa[i]));
                vLimit = Math.min(vLimit, vCentripetal);

                // Angular velocity limit from curvature: ω = v * κ
                double vAngular = c.maxAngVel / Math.abs(kappa[i]);
                vLimit = Math.min(vLimit, vAngular);

                // Mecanum wheel speed feasibility
                double vMecanum = DriveConstants.MAX_WHEEL_SPEED_IN_S / (1.0 + mecanumK * Math.abs(kappa[i]));
                vLimit = Math.min(vLimit, vMecanum);
            }

            // Store maximum velocity
            vMax[i] = Math.max(0.01, vLimit);
            v[i] = vMax[i];
        }

        // ==================== HEADING UNWRAPPING ====================

        // Initialize first heading
        if (unwrapSeedOrNull != null) {
            theta[0] = HeadingUtil.unwrapToNear(theta[0], unwrapSeedOrNull);
        }

        // Unwrap subsequent headings for continuous derivatives
        for (int i = 1; i < N; i++) {
            theta[i] = HeadingUtil.unwrapToNear(theta[i], theta[i - 1]);
        }

        // ==================== HEADING DERIVATIVES ====================

        double[] thetaS = new double[N];   // dθ/ds
        double[] thetaSS = new double[N];  // d²θ/ds²

        // Central differences for interior, one-sided at boundaries
        for (int i = 0; i < N; i++) {
            if (i == 0) {
                double ds_i = Math.max(1e-9, s[1] - s[0]);
                thetaS[i] = (theta[1] - theta[0]) / ds_i;
            } else if (i == N - 1) {
                double ds_i = Math.max(1e-9, s[N - 1] - s[N - 2]);
                thetaS[i] = (theta[N - 1] - theta[N - 2]) / ds_i;
            } else {
                double dsL = Math.max(1e-9, s[i] - s[i - 1]);
                double dsR = Math.max(1e-9, s[i + 1] - s[i]);
                double dL = (theta[i] - theta[i - 1]) / dsL;
                double dR = (theta[i + 1] - theta[i]) / dsR;
                thetaS[i] = (dL + dR) / 2.0;
            }
        }

        // Second derivative
        for (int i = 1; i < N - 1; i++) {
            double dsTotal = Math.max(1e-9, s[i + 1] - s[i - 1]);
            double dsL = Math.max(1e-9, s[i] - s[i - 1]);
            double dsR = Math.max(1e-9, s[i + 1] - s[i]);
            double dL = (theta[i] - theta[i - 1]) / dsL;
            double dR = (theta[i + 1] - theta[i]) / dsR;
            thetaSS[i] = (dR - dL) / (dsTotal / 2.0);
        }
        thetaSS[0] = thetaSS[1];
        thetaSS[N - 1] = thetaSS[N - 2];

        // ==================== APPLY HEADING-BASED VELOCITY LIMITS ====================

        // Only apply if using a custom heading profile (for tangent heading, curvature limit handles it)
        if (headingProfileOrNull != null) {
            for (int i = 0; i < N; i++) {
                if (Math.abs(thetaS[i]) > 1e-9) {
                    // ω = v * dθ/ds ≤ maxAngVel  →  v ≤ maxAngVel / |dθ/ds|
                    double vAngLimit = c.maxAngVel / Math.abs(thetaS[i]);
                    v[i] = Math.min(v[i], vAngLimit);
                }
            }
        }

        // ==================== PASS 2: FORWARD PASS (ACCELERATION LIMITED) ====================

        v[0] = 0.0;  // Start from rest
        a[0] = c.maxAccel;

        for (int i = 1; i < N; i++) {
            double ds_i = Math.max(1e-9, s[i] - s[i - 1]);

            // Calculate maximum acceleration considering jerk limit
            double aPrev = a[i - 1];
            double vPrev = Math.max(0.01, v[i - 1]);

            // Estimate time for this segment
            double dtEst = ds_i / vPrev;

            // Jerk-limited acceleration ramp: a_new ≤ a_prev + jerk * dt
            double aMaxJerk = aPrev + c.maxJerk * dtEst;
            double aMax = Math.min(c.maxAccel, Math.max(0, aMaxJerk));

            // Angular acceleration constraint: |α| = |θ_ss * v² + θ_s * a| ≤ maxAngAccel
            if (Math.abs(thetaS[i]) > 1e-9) {
                double angTermFromV = Math.abs(thetaSS[i]) * vPrev * vPrev;
                double remaining = Math.max(0, c.maxAngAccel - angTermFromV);
                double aAngLimit = remaining / Math.abs(thetaS[i]);
                aMax = Math.min(aMax, aAngLimit);
            }

            // Curvature-based angular acceleration (for tangent heading)
            if (Math.abs(kappa[i]) > 1e-9) {
                double aKappaLimit = c.maxAngAccel / Math.abs(kappa[i]);
                aMax = Math.min(aMax, aKappaLimit);
            }

            // Ensure positive acceleration
            aMax = Math.max(0.1, aMax);

            // Kinematic equation: v² = v₀² + 2*a*ds
            double vFromAccel = Math.sqrt(Math.max(0, vPrev * vPrev + 2.0 * aMax * ds_i));

            // Apply local velocity limit
            v[i] = Math.min(v[i], vFromAccel);
            v[i] = Math.min(v[i], vMax[i]);

            // Store acceleration used
            a[i] = aMax;
        }

        // ==================== PASS 3: BACKWARD PASS (DECELERATION LIMITED) ====================

        v[N - 1] = 0.0;  // End at rest
        double[] aBack = new double[N];
        aBack[N - 1] = c.maxDecel;

        for (int i = N - 2; i >= 0; i--) {
            double ds_i = Math.max(1e-9, s[i + 1] - s[i]);

            // Deceleration constraints
            double aNext = aBack[i + 1];
            double vNext = Math.max(0.01, v[i + 1]);

            double dtEst = ds_i / vNext;
            double aMaxJerk = aNext + c.maxJerk * dtEst;
            double aMax = Math.min(c.maxDecel, Math.max(0, aMaxJerk));

            // Angular deceleration constraint
            if (Math.abs(thetaS[i]) > 1e-9) {
                double angTermFromV = Math.abs(thetaSS[i]) * vNext * vNext;
                double remaining = Math.max(0, c.maxAngAccel - angTermFromV);
                double aAngLimit = remaining / Math.abs(thetaS[i]);
                aMax = Math.min(aMax, aAngLimit);
            }

            if (Math.abs(kappa[i]) > 1e-9) {
                double aKappaLimit = c.maxAngDecel / Math.abs(kappa[i]);
                aMax = Math.min(aMax, aKappaLimit);
            }

            aMax = Math.max(0.1, aMax);

            // Backward kinematic equation
            double vFromDecel = Math.sqrt(Math.max(0, vNext * vNext + 2.0 * aMax * ds_i));

            // Take minimum of forward and backward profiles
            v[i] = Math.min(v[i], vFromDecel);
            aBack[i] = aMax;
        }

        // ==================== ENSURE MINIMUM VELOCITY ====================

        double vMin = 0.5;  // Minimum velocity to avoid infinite time
        for (int i = 0; i < N; i++) {
            // Allow very low velocity only at start and end
            if (i > 0 && i < N - 1) {
                v[i] = Math.max(vMin, v[i]);
            } else {
                v[i] = Math.max(0.01, v[i]);
            }
        }

        // ==================== COMPUTE ACCELERATION FROM VELOCITY PROFILE ====================

        // Recompute acceleration from final velocity profile
        for (int i = 0; i < N; i++) {
            if (i == 0) {
                double ds_i = Math.max(1e-9, s[1] - s[0]);
                a[i] = (v[1] * v[1] - v[0] * v[0]) / (2.0 * ds_i);
            } else {
                double ds_i = Math.max(1e-9, s[i] - s[i - 1]);
                a[i] = (v[i] * v[i] - v[i - 1] * v[i - 1]) / (2.0 * ds_i);
            }
            // Clamp acceleration
            a[i] = clamp(a[i], -c.maxDecel, c.maxAccel);
        }

        // ==================== TIME INTEGRATION ====================

        List<TrajSample> samples = new ArrayList<>(N);
        double t = 0.0;

        // First sample
        double omega0 = thetaS[0] * v[0];
        double alpha0 = thetaSS[0] * v[0] * v[0] + thetaS[0] * a[0];
        samples.add(new TrajSample(
                0.0, s[0], v[0], a[0],
                new Pose2d(pos[0].x, pos[0].y, HeadingUtil.wrap(theta[0])),
                tan[0], kappa[0], omega0, alpha0
        ));

        for (int i = 1; i < N; i++) {
            double ds_i = s[i] - s[i - 1];

            // Time from trapezoidal integration
            double vAvg = (v[i] + v[i - 1]) / 2.0;
            vAvg = Math.max(0.1, vAvg);  // Prevent divide by zero
            double dt = ds_i / vAvg;

            // Sanity check on dt
            if (!Double.isFinite(dt) || dt <= 0) {
                dt = 0.01;
            }
            if (dt > 10.0) {
                dt = 10.0;  // Cap at 10 seconds per segment (something is wrong)
            }

            t += dt;

            // Angular kinematics
            double omega = thetaS[i] * v[i];
            double alpha = thetaSS[i] * v[i] * v[i] + thetaS[i] * a[i];

            samples.add(new TrajSample(
                    t, s[i], v[i], a[i],
                    new Pose2d(pos[i].x, pos[i].y, HeadingUtil.wrap(theta[i])),
                    tan[i], kappa[i], omega, alpha
            ));
        }

        return samples;
    }

    /**
     * Create a stationary trajectory for degenerate paths.
     */
    private static List<TrajSample> createStationaryTrajectory(CompositePath path, HeadingProfile hp) {
        List<TrajSample> samples = new ArrayList<>();
        PathSample ps = path.sampleS(0);
        double heading = (hp != null) ? hp.headingAt(0) : Math.atan2(ps.tangent.y, ps.tangent.x);

        samples.add(new TrajSample(
                0.0, 0.0, 0.0, 0.0,
                new Pose2d(ps.pos.x, ps.pos.y, heading),
                ps.tangent, ps.curvature, 0.0, 0.0
        ));
        samples.add(new TrajSample(
                0.1, 0.0, 0.0, 0.0,
                new Pose2d(ps.pos.x, ps.pos.y, heading),
                ps.tangent, ps.curvature, 0.0, 0.0
        ));

        return samples;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}