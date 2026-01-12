package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite path composed of multiple parametric path segments.
 *
 * Features:
 * - Arc-length parameterization for uniform sampling
 * - Smooth curvature computation at segment boundaries
 * - Binary search for efficient s → position lookup
 * - Configurable sampling resolution
 * - Closest point finding for path re-acquisition
 */
public class CompositePath {

    /**
     * Internal sample point with all geometric information.
     */
    private static class PathPoint {
        final double s;          // Cumulative arc length (inches)
        final int segmentIndex;  // Which segment this point belongs to
        final double u;          // Parameter within segment [0, 1]
        final Vector2d pos;      // Position
        final Vector2d dp;       // First derivative (w.r.t. u)
        final Vector2d ddp;      // Second derivative (w.r.t. u)

        PathPoint(double s, int segmentIndex, double u, Vector2d pos, Vector2d dp, Vector2d ddp) {
            this.s = s;
            this.segmentIndex = segmentIndex;
            this.u = u;
            this.pos = pos;
            this.dp = dp;
            this.ddp = ddp;
        }
    }

    // Path segments
    private final List<ParametricPath> segments = new ArrayList<>();

    // Lookup table for arc-length parameterization
    private final List<PathPoint> table = new ArrayList<>();

    // Cached total length
    private double totalLength = 0.0;

    // ==================== BUILDING THE PATH ====================

    /**
     * Add a parametric path segment.
     */
    public void add(ParametricPath segment) {
        if (segment != null) {
            segments.add(segment);
        }
    }

    /**
     * Add multiple segments at once.
     */
    public void addAll(List<ParametricPath> segs) {
        for (ParametricPath seg : segs) {
            add(seg);
        }
    }

    /**
     * Build the arc-length lookup table with specified samples per segment.
     *
     * @param samplesPerSegment  Number of samples per segment (higher = more accurate)
     */
    public void build(int samplesPerSegment) {
        // Convert to approximate ds
        double totalEstLength = 0;
        for (ParametricPath seg : segments) {
            totalEstLength += seg.length();
        }
        double ds = Math.max(0.1, totalEstLength / Math.max(1, segments.size() * samplesPerSegment));
        buildByDs(ds);
    }

    /**
     * Build the arc-length lookup table with specified arc-length resolution.
     *
     * @param ds  Target distance between samples (inches)
     */
    public void buildByDs(double ds) {
        table.clear();
        totalLength = 0.0;

        if (segments.isEmpty()) {
            return;
        }

        ds = Math.max(0.05, Math.min(2.0, ds));

        for (int segIdx = 0; segIdx < segments.size(); segIdx++) {
            ParametricPath seg = segments.get(segIdx);
            double segLength = Math.max(1e-6, seg.length());

            // Number of samples for this segment
            int numSamples = Math.max(4, (int) Math.ceil(segLength / ds));

            // Sample the segment
            Vector2d prevPos = null;
            for (int i = 0; i <= numSamples; i++) {
                double u = (double) i / numSamples;

                Vector2d pos = seg.p(u);
                Vector2d dp = seg.dp(u);
                Vector2d ddp = seg.ddp(u);

                // Calculate arc length increment
                if (prevPos != null) {
                    double deltaS = pos.minus(prevPos).norm();
                    totalLength += deltaS;
                }

                // Add sample point
                table.add(new PathPoint(totalLength, segIdx, u, pos, dp, ddp));
                prevPos = pos;
            }
        }

        // Ensure we have at least two points
        if (table.size() < 2) {
            if (!segments.isEmpty()) {
                ParametricPath seg = segments.get(0);
                table.clear();
                table.add(new PathPoint(0, 0, 0, seg.p(0), seg.dp(0), seg.ddp(0)));
                table.add(new PathPoint(seg.length(), 0, 1, seg.p(1), seg.dp(1), seg.ddp(1)));
                totalLength = seg.length();
            }
        }
    }

    /**
     * Clear all segments and reset.
     */
    public void clear() {
        segments.clear();
        table.clear();
        totalLength = 0.0;
    }

    /**
     * Get total path length in inches.
     */
    public double length() {
        return totalLength;
    }

    /**
     * Get number of segments.
     */
    public int numSegments() {
        return segments.size();
    }

    /**
     * Check if path has been built.
     */
    public boolean isBuilt() {
        return !table.isEmpty();
    }

    // ==================== SAMPLING ====================

    /**
     * Sample the path at arc-length s.
     *
     * @param s  Arc length from start (inches)
     * @return PathSample with position, tangent, and curvature
     */
    public PathSample sampleS(double s) {
        if (table.isEmpty()) {
            throw new IllegalStateException("CompositePath not built - call build() or buildByDs() first");
        }

        // Clamp to valid range
        if (s <= 0) {
            return createSample(table.get(0), 0);
        }
        if (s >= totalLength) {
            return createSample(table.get(table.size() - 1), totalLength);
        }

        // Binary search for bracketing points
        int lo = 0;
        int hi = table.size() - 1;

        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (table.get(mid).s < s) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        PathPoint a = table.get(lo);
        PathPoint b = table.get(hi);

        // Interpolation factor
        double span = Math.max(1e-9, b.s - a.s);
        double t = (s - a.s) / span;
        t = Math.max(0, Math.min(1, t));

        // Linear interpolation of position
        Vector2d pos = new Vector2d(
                a.pos.x + (b.pos.x - a.pos.x) * t,
                a.pos.y + (b.pos.y - a.pos.y) * t
        );

        // Linear interpolation of derivatives
        Vector2d dp = new Vector2d(
                a.dp.x + (b.dp.x - a.dp.x) * t,
                a.dp.y + (b.dp.y - a.dp.y) * t
        );

        Vector2d ddp = new Vector2d(
                a.ddp.x + (b.ddp.x - a.ddp.x) * t,
                a.ddp.y + (b.ddp.y - a.ddp.y) * t
        );

        // Compute tangent and curvature
        Vector2d tangent = computeTangent(dp, a.dp);
        double curvature = computeCurvature(dp, ddp);

        return new PathSample(s, pos, tangent, curvature);
    }

    /**
     * Find the arc length s of the point on the path closest to the given position.
     * Uses a two-phase search: coarse table scan, then fine binary refinement.
     *
     * @param point  Query point
     * @return Arc length of closest point
     */
    public double closestS(Vector2d point) {
        if (table.isEmpty()) {
            return 0;
        }

        double bestS = 0;
        double bestDist2 = Double.POSITIVE_INFINITY;

        // Phase 1: Coarse search through table
        for (PathPoint pp : table) {
            double dx = pp.pos.x - point.x;
            double dy = pp.pos.y - point.y;
            double dist2 = dx * dx + dy * dy;

            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestS = pp.s;
            }
        }

        // Phase 2: Fine search around best point using golden section
        double searchRadius = Math.min(5.0, totalLength * 0.05);
        double sLo = Math.max(0, bestS - searchRadius);
        double sHi = Math.min(totalLength, bestS + searchRadius);

        // Golden section search for minimum distance
        final double phi = (Math.sqrt(5) - 1) / 2;  // Golden ratio conjugate
        double a = sLo, b = sHi;
        double c = b - phi * (b - a);
        double d = a + phi * (b - a);

        for (int iter = 0; iter < 20; iter++) {
            double fc = distanceSquaredAt(c, point);
            double fd = distanceSquaredAt(d, point);

            if (fc < fd) {
                b = d;
                d = c;
                c = b - phi * (b - a);
            } else {
                a = c;
                c = d;
                d = a + phi * (b - a);
            }

            if (Math.abs(b - a) < 0.01) break;
        }

        bestS = (a + b) / 2;

        // Final check: is refined result actually better?
        double refinedDist2 = distanceSquaredAt(bestS, point);
        if (refinedDist2 > bestDist2) {
            // Keep original coarse result
            for (PathPoint pp : table) {
                double dx = pp.pos.x - point.x;
                double dy = pp.pos.y - point.y;
                double dist2 = dx * dx + dy * dy;
                if (dist2 <= bestDist2) {
                    bestS = pp.s;
                    break;
                }
            }
        }

        return bestS;
    }

    /**
     * Helper: compute squared distance from point to path at arc-length s.
     */
    private double distanceSquaredAt(double s, Vector2d point) {
        PathSample ps = sampleS(s);
        double dx = ps.pos.x - point.x;
        double dy = ps.pos.y - point.y;
        return dx * dx + dy * dy;
    }

    /**
     * Get the starting position of the path.
     */
    public Vector2d startPosition() {
        if (table.isEmpty()) {
            return new Vector2d(0, 0);
        }
        return table.get(0).pos;
    }

    /**
     * Get the ending position of the path.
     */
    public Vector2d endPosition() {
        if (table.isEmpty()) {
            return new Vector2d(0, 0);
        }
        return table.get(table.size() - 1).pos;
    }

    /**
     * Get starting tangent direction.
     */
    public Vector2d startTangent() {
        if (table.isEmpty()) {
            return new Vector2d(1, 0);
        }
        return computeTangent(table.get(0).dp, null);
    }

    /**
     * Get ending tangent direction.
     */
    public Vector2d endTangent() {
        if (table.isEmpty()) {
            return new Vector2d(1, 0);
        }
        return computeTangent(table.get(table.size() - 1).dp, null);
    }

    // ==================== HELPER METHODS ====================

    /**
     * Create a PathSample from a PathPoint.
     */
    private PathSample createSample(PathPoint pp, double s) {
        Vector2d tangent = computeTangent(pp.dp, null);
        double curvature = computeCurvature(pp.dp, pp.ddp);
        return new PathSample(s, pp.pos, tangent, curvature);
    }

    /**
     * Compute unit tangent vector from derivative.
     * Falls back to fallback vector if derivative is too small.
     */
    private Vector2d computeTangent(Vector2d dp, Vector2d fallback) {
        double norm = dp.norm();
        if (norm > 1e-6) {
            return new Vector2d(dp.x / norm, dp.y / norm);
        }
        if (fallback != null) {
            norm = fallback.norm();
            if (norm > 1e-6) {
                return new Vector2d(fallback.x / norm, fallback.y / norm);
            }
        }
        return new Vector2d(1, 0);  // Default: pointing in +X
    }

    /**
     * Compute signed curvature from first and second derivatives.
     *
     * Curvature κ = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
     *
     * Positive curvature = turning left (CCW)
     * Negative curvature = turning right (CW)
     */
    private double computeCurvature(Vector2d dp, Vector2d ddp) {
        double x1 = dp.x, y1 = dp.y;
        double x2 = ddp.x, y2 = ddp.y;

        double speedSq = x1 * x1 + y1 * y1;
        if (speedSq < 1e-12) {
            return 0.0;  // Undefined curvature at zero velocity
        }

        double cross = x1 * y2 - y1 * x2;
        double denom = Math.pow(speedSq, 1.5);

        return cross / denom;
    }

    /**
     * Get all segments (for advanced use).
     */
    public List<ParametricPath> getSegments() {
        return new ArrayList<>(segments);
    }

    /**
     * Sample multiple points along the path at regular arc-length intervals.
     * Useful for visualization or analysis.
     *
     * @param numPoints  Number of points to sample
     * @return List of PathSamples
     */
    public List<PathSample> sampleUniform(int numPoints) {
        List<PathSample> samples = new ArrayList<>();
        numPoints = Math.max(2, numPoints);

        for (int i = 0; i < numPoints; i++) {
            double s = ((double) i / (numPoints - 1)) * totalLength;
            samples.add(sampleS(s));
        }

        return samples;
    }

    /**
     * Get the curvature at a specific arc length.
     */
    public double curvatureAt(double s) {
        return sampleS(s).curvature;
    }

    /**
     * Get the maximum absolute curvature along the path.
     * Useful for determining velocity limits.
     */
    public double maxAbsCurvature() {
        double maxK = 0;
        for (PathPoint pp : table) {
            double k = Math.abs(computeCurvature(pp.dp, pp.ddp));
            if (k > maxK) maxK = k;
        }
        return maxK;
    }

    /**
     * Check if path is valid (non-empty and built).
     */
    public boolean isValid() {
        return !segments.isEmpty() && !table.isEmpty() && totalLength > 0;
    }
}