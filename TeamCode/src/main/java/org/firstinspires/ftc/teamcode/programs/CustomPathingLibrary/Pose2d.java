package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * 2D pose (position + heading) for robot state.
 *
 * Convention:
 * - x, y: Position in inches
 * - heading: Orientation in radians, CCW positive from +X axis
 */
public class Pose2d {
    public double x;
    public double y;
    public double heading;  // radians

    // ==================== CONSTRUCTORS ====================

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    /**
     * Create from Vector2d position and heading.
     */
    public Pose2d(Vector2d position, double heading) {
        this(position.x, position.y, heading);
    }

    // ==================== POSITION ACCESS ====================

    /**
     * Get position as Vector2d.
     */
    public Vector2d position() {
        return new Vector2d(x, y);
    }

    /**
     * Get heading vector (unit vector pointing in heading direction).
     */
    public Vector2d headingVector() {
        return new Vector2d(Math.cos(heading), Math.sin(heading));
    }

    // ==================== OPERATIONS ====================

    /**
     * Create a copy.
     */
    public Pose2d copy() {
        return new Pose2d(x, y, heading);
    }

    /**
     * Add another pose (position adds, heading adds and wraps).
     */
    public Pose2d plus(Pose2d other) {
        return new Pose2d(
                x + other.x,
                y + other.y,
                HeadingUtil.wrap(heading + other.heading)
        );
    }

    /**
     * Subtract another pose.
     */
    public Pose2d minus(Pose2d other) {
        return new Pose2d(
                x - other.x,
                y - other.y,
                HeadingUtil.wrap(heading - other.heading)
        );
    }

    /**
     * Transform a vector from robot frame to field frame.
     * Rotates the vector by this pose's heading and translates by position.
     */
    public Vector2d transformToField(Vector2d robotPoint) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        return new Vector2d(
                x + robotPoint.x * cos - robotPoint.y * sin,
                y + robotPoint.x * sin + robotPoint.y * cos
        );
    }

    /**
     * Transform a vector from field frame to robot frame.
     */
    public Vector2d transformToRobot(Vector2d fieldPoint) {
        double dx = fieldPoint.x - x;
        double dy = fieldPoint.y - y;

        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        return new Vector2d(
                dx * cos - dy * sin,
                dx * sin + dy * cos
        );
    }

    // ==================== DISTANCE ====================

    /**
     * Euclidean distance to another pose (position only).
     */
    public double distanceTo(Pose2d other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.hypot(dx, dy);
    }

    /**
     * Distance to a point.
     */
    public double distanceTo(Vector2d point) {
        return Math.hypot(x - point.x, y - point.y);
    }

    /**
     * Heading difference to another pose (shortest angle).
     */
    public double headingDeltaTo(Pose2d other) {
        return HeadingUtil.shortestDelta(heading, other.heading);
    }

    // ==================== INTERPOLATION ====================

    /**
     * Linear interpolation between poses.
     * Uses shortest-path interpolation for heading.
     */
    public Pose2d lerp(Pose2d other, double t) {
        return new Pose2d(
                x + (other.x - x) * t,
                y + (other.y - y) * t,
                HeadingUtil.lerp(heading, other.heading, t)
        );
    }

    // ==================== UTILITY ====================

    /**
     * Check if approximately equal to another pose.
     */
    public boolean epsilonEquals(Pose2d other, double posEpsilon, double headingEpsilon) {
        double posDist = distanceTo(other);
        double headDiff = Math.abs(HeadingUtil.shortestDelta(heading, other.heading));
        return posDist < posEpsilon && headDiff < headingEpsilon;
    }

    /**
     * Get heading in degrees.
     */
    public double headingDegrees() {
        return Math.toDegrees(heading);
    }

    /**
     * Create pose with heading in degrees (converts to radians internally).
     */
    public static Pose2d fromDegrees(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    @Override
    public String toString() {
        return String.format("Pose2d(%.2f, %.2f, %.1f°)", x, y, Math.toDegrees(heading));
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Pose2d)) return false;
        Pose2d other = (Pose2d) obj;
        return Double.compare(x, other.x) == 0 &&
                Double.compare(y, other.y) == 0 &&
                Double.compare(heading, other.heading) == 0;
    }

    @Override
    public int hashCode() {
        long xBits = Double.doubleToLongBits(x);
        long yBits = Double.doubleToLongBits(y);
        long hBits = Double.doubleToLongBits(heading);
        int result = (int) (xBits ^ (xBits >>> 32));
        result = 31 * result + (int) (yBits ^ (yBits >>> 32));
        result = 31 * result + (int) (hBits ^ (hBits >>> 32));
        return result;
    }
}