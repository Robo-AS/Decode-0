package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Heading profile that interpolates from start heading to end heading.
 *
 * Uses shortest-angle interpolation to avoid unnecessary rotation.
 * Optional smooth easing for reduced jerk at start/end.
 */
public class FixedStartEndHeading implements HeadingProfile {

    private final double startHeading;
    private final double endHeading;
    private final double totalLength;
    private final boolean smooth;

    /**
     * Create linear interpolation profile.
     *
     * @param startHeading  Starting heading (radians)
     * @param endHeading    Ending heading (radians)
     * @param totalLength   Total path length (inches)
     */
    public FixedStartEndHeading(double startHeading, double endHeading, double totalLength) {
        this(startHeading, endHeading, totalLength, false);
    }

    /**
     * Create interpolation profile with optional smoothing.
     *
     * @param startHeading  Starting heading (radians)
     * @param endHeading    Ending heading (radians)
     * @param totalLength   Total path length (inches)
     * @param smooth        If true, use S-curve easing for smoother transitions
     */
    public FixedStartEndHeading(double startHeading, double endHeading, double totalLength, boolean smooth) {
        this.startHeading = HeadingUtil.wrap(startHeading);
        this.endHeading = HeadingUtil.wrap(endHeading);
        this.totalLength = Math.max(1e-6, totalLength);
        this.smooth = smooth;
    }

    @Override
    public double headingAt(double s) {
        // Calculate interpolation factor [0, 1]
        double t = s / totalLength;
        t = HeadingUtil.clamp01(t);

        // Apply smoothstep if requested
        if (smooth) {
            t = HeadingUtil.smooth01(t);
        }

        // Interpolate using shortest angle
        double delta = HeadingUtil.shortestDelta(startHeading, endHeading);
        return HeadingUtil.wrap(startHeading + delta * t);
    }

    /**
     * Get the total heading change (radians, signed).
     */
    public double getTotalChange() {
        return HeadingUtil.shortestDelta(startHeading, endHeading);
    }

    /**
     * Get start heading.
     */
    public double getStartHeading() {
        return startHeading;
    }

    /**
     * Get end heading.
     */
    public double getEndHeading() {
        return endHeading;
    }

    /**
     * Get total path length.
     */
    public double getTotalLength() {
        return totalLength;
    }

    /**
     * Check if smooth interpolation is enabled.
     */
    public boolean isSmooth() {
        return smooth;
    }

    @Override
    public String toString() {
        return String.format("FixedStartEndHeading(%.1f° -> %.1f° over %.1f in, smooth=%b)",
                Math.toDegrees(startHeading), Math.toDegrees(endHeading), totalLength, smooth);
    }
}