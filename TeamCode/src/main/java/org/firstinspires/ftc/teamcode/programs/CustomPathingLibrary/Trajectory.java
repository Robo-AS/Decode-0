package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.List;

/**
 * Time-parameterized trajectory for path following.
 *
 * Contains a sequence of States sampled over time, with interpolation
 * for querying at arbitrary times.
 *
 * Features:
 * - Efficient binary search for time lookup
 * - Proper angle interpolation (shortest path)
 * - Tangent vector renormalization
 * - Closest point finding for path re-acquisition
 */
public class Trajectory {

    /**
     * A single state along the trajectory.
     */
    public static class State {
        public final double t;          // Time (seconds)
        public final double s;          // Arc length (inches)
        public final double v;          // Linear velocity (in/s)
        public final double a;          // Linear acceleration (in/s²)
        public final Pose2d pose;       // Position and heading
        public final Vector2d tangent;  // Unit tangent vector (path direction)
        public final double curvature;  // Path curvature (1/in)
        public final double omega;      // Angular velocity (rad/s)
        public final double alpha;      // Angular acceleration (rad/s²)

        public State(double t, double s, double v, double a,
                     Pose2d pose, Vector2d tangent,
                     double curvature, double omega, double alpha) {
            this.t = t;
            this.s = s;
            this.v = v;
            this.a = a;
            this.pose = pose;
            this.tangent = (tangent != null && tangent.norm() > 1e-6)
                    ? tangent.normalized()
                    : new Vector2d(1, 0);
            this.curvature = curvature;
            this.omega = omega;
            this.alpha = alpha;
        }

        /**
         * Get the heading in radians.
         */
        public double getHeading() {
            return pose.heading;
        }

        /**
         * Get position as Vector2d.
         */
        public Vector2d getPosition() {
            return new Vector2d(pose.x, pose.y);
        }

        @Override
        public String toString() {
            return String.format("State(t=%.2f, s=%.1f, v=%.1f, pos=(%.1f,%.1f), h=%.0f°)",
                    t, s, v, pose.x, pose.y, Math.toDegrees(pose.heading));
        }
    }

    // ==================== FIELDS ====================

    private final List<State> states;
    private final double duration;

    // Cached start and end states
    private final State startState;
    private final State endState;

    // ==================== CONSTRUCTOR ====================

    /**
     * Create a trajectory from a list of states.
     * States must be sorted by time in ascending order.
     *
     * @param states  List of trajectory states (must not be null or empty)
     */
    public Trajectory(List<State> states) {
        if (states == null || states.isEmpty()) {
            throw new IllegalArgumentException("Trajectory states cannot be null or empty");
        }

        this.states = states;
        this.startState = states.get(0);
        this.endState = states.get(states.size() - 1);
        this.duration = endState.t - startState.t;

        // Validate time ordering
        for (int i = 1; i < states.size(); i++) {
            if (states.get(i).t < states.get(i - 1).t) {
                throw new IllegalArgumentException("Trajectory states must be sorted by time");
            }
        }
    }

    // ==================== BASIC PROPERTIES ====================

    /**
     * Get total trajectory duration in seconds.
     */
    public double duration() {
        return duration;
    }

    /**
     * Get total path length in inches.
     */
    public double length() {
        return endState.s - startState.s;
    }

    /**
     * Get all states (for iteration or analysis).
     */
    public List<State> allStates() {
        return states;
    }

    /**
     * Get number of states.
     */
    public int size() {
        return states.size();
    }

    /**
     * Get the starting state.
     */
    public State start() {
        return startState;
    }

    /**
     * Get the ending state.
     */
    public State end() {
        return endState;
    }

    // ==================== SAMPLING BY TIME ====================

    /**
     * Sample the trajectory at a given time.
     * Uses binary search and linear interpolation between states.
     *
     * @param t  Time in seconds from trajectory start
     * @return Interpolated state at time t
     */
    public State sample(double t) {
        // Handle boundary cases
        if (t <= 0) {
            return startState;
        }
        if (t >= duration) {
            return endState;
        }

        // Binary search for bracketing states
        int lo = 0;
        int hi = states.size() - 1;

        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (states.get(mid).t < t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        State a = states.get(lo);
        State b = states.get(hi);

        // Interpolation factor
        double dt = Math.max(1e-9, b.t - a.t);
        double u = (t - a.t) / dt;
        u = Math.max(0, Math.min(1, u));

        return interpolate(a, b, u);
    }

    /**
     * Interpolate between two states.
     */
    private State interpolate(State a, State b, double u) {
        // Linear interpolation for most values
        double t = lerp(a.t, b.t, u);
        double s = lerp(a.s, b.s, u);
        double v = lerp(a.v, b.v, u);
        double acc = lerp(a.a, b.a, u);
        double curvature = lerp(a.curvature, b.curvature, u);
        double omega = lerp(a.omega, b.omega, u);
        double alpha = lerp(a.alpha, b.alpha, u);

        // Position interpolation
        double x = lerp(a.pose.x, b.pose.x, u);
        double y = lerp(a.pose.y, b.pose.y, u);

        // Heading interpolation (shortest angle)
        double heading = lerpAngle(a.pose.heading, b.pose.heading, u);

        // Tangent interpolation (renormalize)
        double tx = lerp(a.tangent.x, b.tangent.x, u);
        double ty = lerp(a.tangent.y, b.tangent.y, u);
        double tn = Math.hypot(tx, ty);
        Vector2d tangent;
        if (tn > 1e-6) {
            tangent = new Vector2d(tx / tn, ty / tn);
        } else {
            tangent = a.tangent;  // Fallback to previous tangent
        }

        return new State(t, s, v, acc, new Pose2d(x, y, heading), tangent, curvature, omega, alpha);
    }

    // ==================== CLOSEST POINT FINDING ====================

    /**
     * Find the time t of the state closest to a given pose.
     * Useful for path re-acquisition after disturbances.
     *
     * @param p  Query pose
     * @return Time of closest state
     */
    public double closestTimeTo(Pose2d p) {
        if (states.isEmpty()) return 0.0;

        double bestT = startState.t;
        double bestDist2 = Double.POSITIVE_INFINITY;

        // Coarse search through all states
        for (State state : states) {
            double dx = state.pose.x - p.x;
            double dy = state.pose.y - p.y;
            double dist2 = dx * dx + dy * dy;

            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestT = state.t;
            }
        }

        // Fine search around best point
        double searchRadius = Math.min(0.5, duration * 0.1);
        double tLo = Math.max(0, bestT - searchRadius);
        double tHi = Math.min(duration, bestT + searchRadius);

        // Golden section search
        final double phi = (Math.sqrt(5) - 1) / 2;
        double a = tLo, b = tHi;
        double c = b - phi * (b - a);
        double d = a + phi * (b - a);

        for (int iter = 0; iter < 15; iter++) {
            double fc = distanceSquaredAt(c, p);
            double fd = distanceSquaredAt(d, p);

            if (fc < fd) {
                b = d;
                d = c;
                c = b - phi * (b - a);
            } else {
                a = c;
                c = d;
                d = a + phi * (b - a);
            }

            if (Math.abs(b - a) < 0.005) break;
        }

        return (a + b) / 2;
    }

    /**
     * Find the time t of the state closest to a given position (ignoring heading).
     */
    public double closestTimeTo(Vector2d position) {
        return closestTimeTo(new Pose2d(position.x, position.y, 0));
    }

    /**
     * Helper: compute squared distance from pose to trajectory at time t.
     */
    private double distanceSquaredAt(double t, Pose2d p) {
        State state = sample(t);
        double dx = state.pose.x - p.x;
        double dy = state.pose.y - p.y;
        return dx * dx + dy * dy;
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Linear interpolation.
     */
    private static double lerp(double a, double b, double u) {
        return a + (b - a) * u;
    }

    /**
     * Angle interpolation using shortest path.
     */
    private static double lerpAngle(double a, double b, double u) {
        // Normalize both angles
        a = normalizeAngle(a);
        b = normalizeAngle(b);

        // Find shortest delta
        double delta = normalizeAngle(b - a);

        // Interpolate
        return normalizeAngle(a + delta * u);
    }

    /**
     * Normalize angle to (-π, π].
     */
    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /**
     * Check if trajectory is valid.
     */
    public boolean isValid() {
        return states != null && !states.isEmpty() && duration >= 0;
    }

    /**
     * Get a string representation for debugging.
     */
    @Override
    public String toString() {
        return String.format("Trajectory(duration=%.2fs, length=%.1fin, states=%d)",
                duration, length(), states.size());
    }

    /**
     * Get the state at a specific arc length (useful for some algorithms).
     */
    public State sampleByArcLength(double s) {
        // Find state with closest arc length
        if (s <= startState.s) return startState;
        if (s >= endState.s) return endState;

        int lo = 0, hi = states.size() - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (states.get(mid).s < s) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        State a = states.get(lo);
        State b = states.get(hi);

        double ds = Math.max(1e-9, b.s - a.s);
        double u = (s - a.s) / ds;
        u = Math.max(0, Math.min(1, u));

        return interpolate(a, b, u);
    }
}