package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Interpolates heading from startH to endH over arc-length [0, totalS].
 * - Uses shortest-angle interpolation (no wrap jumps).
 * - Optional smooth easing (3t^2 - 2t^3) to reduce jerk at ends.
 */
public class FixedStartEndHeading implements HeadingProfile {
    private final double startH, endH;
    private final double totalS;
    private final boolean smooth;

    /**
     * Linear interpolation version (default).
     */
    public FixedStartEndHeading(double startH, double endH, double totalS){
        this(startH, endH, totalS, false);
    }

    /**
     * @param smooth if true, uses S-curve easing (3t^2 - 2t^3); else linear.
     */
    public FixedStartEndHeading(double startH, double endH, double totalS, boolean smooth){
        this.startH = HeadingUtil.wrap(startH);
        this.endH   = HeadingUtil.wrap(endH);
        this.totalS = Math.max(1e-9, totalS);
        this.smooth = smooth;
    }

    @Override
    public double headingAt(double s){
        double t = HeadingUtil.clamp01(s / totalS);
        if (smooth) t = HeadingUtil.smooth01(t);
        double dh = HeadingUtil.shortestDelta(startH, endH);
        return HeadingUtil.wrap(startH + dh * t);
    }
}