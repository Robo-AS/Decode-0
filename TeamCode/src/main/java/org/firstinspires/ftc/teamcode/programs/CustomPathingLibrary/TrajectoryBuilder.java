package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder for constructing trajectories from path segments.
 *
 * Supports:
 * - Line segments
 * - Circular arcs
 * - Cubic Bezier curves
 * - Spline paths (automatic control point generation)
 * - Multiple heading modes (tangent, fixed, linear interpolation)
 *
 * Usage:
 *   Trajectory traj = new TrajectoryBuilder()
 *       .line(start, end)
 *       .bezier(p0, p1, p2, p3)
 *       .buildTangentHeading(constraints);
 */
public class TrajectoryBuilder {

    // Default sampling resolution
    public static final double DEFAULT_DS = 0.5;  // inches

    // Limits for samples per segment
    private static final int MIN_SPS = 20;
    private static final int MAX_SPS = 500;

    // Internal composite path
    private final CompositePath path = new CompositePath();

    // Track last endpoint for continuity checking
    private Vector2d lastEndpoint = null;
    private boolean warnOnDiscontinuity = true;

    // ==================== PATH SEGMENT BUILDERS ====================

    /**
     * Add a straight line segment from a to b.
     */
    public TrajectoryBuilder line(Vector2d a, Vector2d b) {
        checkContinuity(a);
        path.add(new LinePath(a, b));
        lastEndpoint = b;
        return this;
    }

    /**
     * Add a line from current position to target.
     * Requires at least one previous segment.
     */
    public TrajectoryBuilder lineTo(Vector2d target) {
        if (lastEndpoint == null) {
            throw new IllegalStateException("lineTo() requires a previous segment. Use line() first.");
        }
        return line(lastEndpoint, target);
    }

    /**
     * Add a circular arc.
     *
     * @param center  Center of the circle
     * @param radius  Radius of the arc
     * @param startAngle  Starting angle (radians, CCW from +X)
     * @param endAngle  Ending angle (radians)
     */
    public TrajectoryBuilder arc(Vector2d center, double radius, double startAngle, double endAngle) {
        Vector2d arcStart = new Vector2d(
                center.x + radius * Math.cos(startAngle),
                center.y + radius * Math.sin(startAngle)
        );
        checkContinuity(arcStart);

        path.add(new CircularArcPath(center, radius, startAngle, endAngle));

        lastEndpoint = new Vector2d(
                center.x + radius * Math.cos(endAngle),
                center.y + radius * Math.sin(endAngle)
        );
        return this;
    }

    /**
     * Add a cubic Bezier curve with four control points.
     *
     * @param p0  Start point (on curve)
     * @param p1  First control point (off curve, defines start tangent)
     * @param p2  Second control point (off curve, defines end tangent)
     * @param p3  End point (on curve)
     */
    public TrajectoryBuilder bezier(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3) {
        checkContinuity(p0);
        path.add(new BezierPath(p0, p1, p2, p3));
        lastEndpoint = p3;
        return this;
    }

    /**
     * Add a smooth curve from current position to target with automatic control points.
     * Creates a bezier that starts tangent to the previous segment direction.
     *
     * @param target  End position
     * @param controlDistance  Distance to control points (typically 1/3 of total distance)
     */
    public TrajectoryBuilder splineTo(Vector2d target, double controlDistance) {
        if (lastEndpoint == null) {
            throw new IllegalStateException("splineTo() requires a previous segment");
        }

        // Get tangent direction at end of last segment
        Vector2d tangent = getLastTangent();

        // Control points
        Vector2d p0 = lastEndpoint;
        Vector2d p1 = new Vector2d(
                p0.x + tangent.x * controlDistance,
                p0.y + tangent.y * controlDistance
        );

        // Direction from p1 to target for smooth arrival
        double dx = target.x - p0.x;
        double dy = target.y - p0.y;
        double dist = Math.hypot(dx, dy);

        Vector2d p2 = new Vector2d(
                target.x - (dx / dist) * controlDistance,
                target.y - (dy / dist) * controlDistance
        );

        return bezier(p0, p1, p2, target);
    }

    /**
     * Add a smooth S-curve from current position to target.
     * Useful for lane changes or parallel offsets.
     */
    public TrajectoryBuilder sCurveTo(Vector2d target) {
        if (lastEndpoint == null) {
            throw new IllegalStateException("sCurveTo() requires a previous segment");
        }

        double dist = lastEndpoint.minus(target).norm();
        return splineTo(target, dist / 3.0);
    }

    /**
     * Add a forward movement (in the current tangent direction).
     */
    public TrajectoryBuilder forward(double distance) {
        if (lastEndpoint == null) {
            throw new IllegalStateException("forward() requires a previous segment");
        }

        Vector2d tangent = getLastTangent();
        Vector2d target = new Vector2d(
                lastEndpoint.x + tangent.x * distance,
                lastEndpoint.y + tangent.y * distance
        );

        return line(lastEndpoint, target);
    }

    /**
     * Add a strafe movement (perpendicular to current direction).
     * Positive = left, negative = right.
     */
    public TrajectoryBuilder strafe(double distance) {
        if (lastEndpoint == null) {
            throw new IllegalStateException("strafe() requires a previous segment");
        }

        Vector2d tangent = getLastTangent();
        // Normal vector (90° CCW from tangent)
        Vector2d normal = new Vector2d(-tangent.y, tangent.x);

        Vector2d target = new Vector2d(
                lastEndpoint.x + normal.x * distance,
                lastEndpoint.y + normal.y * distance
        );

        return line(lastEndpoint, target);
    }

    // ==================== TRAJECTORY BUILDING ====================

    /**
     * Build trajectory with heading following path tangent.
     */
    public Trajectory buildTangentHeading(TrajectoryConstraints constraints) {
        return buildTangentHeading(constraints, DEFAULT_DS, null);
    }

    /**
     * Build trajectory with heading following path tangent.
     *
     * @param constraints  Kinematic constraints
     * @param ds  Arc-length sampling resolution (inches)
     * @param headingSeedRad  Initial heading for unwrapping (null to auto-detect)
     */
    public Trajectory buildTangentHeading(TrajectoryConstraints constraints, double ds, Double headingSeedRad) {
        validatePath();

        // Build path with specified resolution
        path.buildByDs(ds);

        // Parameterize with tangent heading
        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, constraints, ds, null, headingSeedRad);

        return createTrajectory(samples);
    }

    /**
     * Build trajectory with custom heading profile.
     */
    public Trajectory buildWithHeading(TrajectoryConstraints constraints, HeadingProfile headingProfile) {
        return buildWithHeading(constraints, DEFAULT_DS, headingProfile, null);
    }

    /**
     * Build trajectory with custom heading profile.
     *
     * @param constraints  Kinematic constraints
     * @param ds  Arc-length sampling resolution
     * @param headingProfile  Custom heading profile
     * @param headingSeedRad  Initial heading for unwrapping
     */
    public Trajectory buildWithHeading(TrajectoryConstraints constraints, double ds,
                                       HeadingProfile headingProfile, Double headingSeedRad) {
        if (headingProfile == null) {
            throw new IllegalArgumentException("Heading profile cannot be null. Use buildTangentHeading() instead.");
        }

        validatePath();
        path.buildByDs(ds);

        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, constraints, ds, headingProfile, headingSeedRad);

        return createTrajectory(samples);
    }

    /**
     * Build trajectory with fixed heading (robot maintains constant orientation).
     *
     * @param constraints  Kinematic constraints
     * @param headingRad  Fixed heading in radians
     */
    public Trajectory buildFixedHeading(TrajectoryConstraints constraints, double headingRad) {
        validatePath();
        path.buildByDs(DEFAULT_DS);

        HeadingProfile fixedProfile = s -> headingRad;

        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, constraints, DEFAULT_DS, fixedProfile, headingRad);

        return createTrajectory(samples);
    }

    /**
     * Build trajectory with linear heading interpolation from start to end.
     */
    public Trajectory buildLinearHeading(TrajectoryConstraints constraints,
                                         double startHeadingRad, double endHeadingRad) {
        validatePath();
        path.buildByDs(DEFAULT_DS);

        HeadingProfile linearProfile = new FixedStartEndHeading(startHeadingRad, endHeadingRad, path.length());

        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, constraints, DEFAULT_DS, linearProfile, startHeadingRad);

        return createTrajectory(samples);
    }

    // ==================== HELPER METHODS ====================

    /**
     * Convert TimeParameterizer samples to Trajectory.
     */
    private Trajectory createTrajectory(List<TimeParameterizer.TrajSample> samples) {
        List<Trajectory.State> states = new ArrayList<>(samples.size());

        for (TimeParameterizer.TrajSample ts : samples) {
            states.add(new Trajectory.State(
                    ts.t, ts.s, ts.v, ts.a,
                    ts.pose, ts.tangent, ts.curvature,
                    ts.omega, ts.alpha
            ));
        }

        return new Trajectory(states);
    }

    /**
     * Validate that path is ready for building.
     */
    private void validatePath() {
        if (path.numSegments() == 0) {
            throw new IllegalStateException("Cannot build trajectory: no path segments added");
        }
    }

    /**
     * Check path continuity when adding a new segment.
     */
    private void checkContinuity(Vector2d newStart) {
        if (lastEndpoint != null && warnOnDiscontinuity) {
            double gap = lastEndpoint.minus(newStart).norm();
            if (gap > 0.1) {  // More than 0.1 inch gap
                // Log warning (in FTC, could use telemetry)
                System.err.printf("TrajectoryBuilder: Path discontinuity of %.2f inches detected%n", gap);
            }
        }
    }

    /**
     * Get tangent direction at end of current path.
     */
    private Vector2d getLastTangent() {
        if (path.numSegments() == 0 || !path.isBuilt()) {
            // Build temporarily to get tangent
            if (path.numSegments() > 0) {
                path.buildByDs(DEFAULT_DS);
                return path.endTangent();
            }
            return new Vector2d(1, 0);  // Default: +X direction
        }
        return path.endTangent();
    }

    /**
     * Clear builder for reuse.
     */
    public TrajectoryBuilder clear() {
        path.clear();
        lastEndpoint = null;
        return this;
    }

    /**
     * Set whether to warn on path discontinuities.
     */
    public TrajectoryBuilder setWarnOnDiscontinuity(boolean warn) {
        this.warnOnDiscontinuity = warn;
        return this;
    }

    /**
     * Get current path length (before time parameterization).
     */
    public double currentPathLength() {
        if (!path.isBuilt()) {
            path.buildByDs(DEFAULT_DS);
        }
        return path.length();
    }

    /**
     * Get number of segments added.
     */
    public int numSegments() {
        return path.numSegments();
    }

    /**
     * Get the starting position (null if no segments added).
     */
    public Vector2d getStartPosition() {
        if (path.numSegments() == 0) return null;
        if (!path.isBuilt()) path.buildByDs(DEFAULT_DS);
        return path.startPosition();
    }

    /**
     * Get the ending position (null if no segments added).
     */
    public Vector2d getEndPosition() {
        return lastEndpoint;
    }

    /**
     * Helper to calculate samples per segment from ds.
     */
    private static int spsFromDs(double ds) {
        ds = Math.max(0.25, Math.min(2.0, ds));
        int sps = (int) Math.round(200.0 / ds);
        return Math.max(MIN_SPS, Math.min(MAX_SPS, sps));
    }
}