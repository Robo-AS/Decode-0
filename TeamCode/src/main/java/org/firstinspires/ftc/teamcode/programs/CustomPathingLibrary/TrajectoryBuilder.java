package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** Build a CompositePath from primitives, time-parameterize it, and inject a heading profile. */
public class TrajectoryBuilder {
    /** Default spatial sampling step along path (inches). */
    public static final double DEFAULT_DS = 0.5;

    /** Reasonable bounds for converting ds → samplesPerSegment in CompositePath.build(int). */
    private static final int MIN_SPS = 50;    // coarse lower bound for long/simple segments
    private static final int MAX_SPS = 600;   // upper bound to cap CPU

    private final CompositePath path = new CompositePath();

    // --- primitives ---
    public TrajectoryBuilder line(Vector2d a, Vector2d b){
        path.add(new LinePath(a,b));
        return this;
    }
    public TrajectoryBuilder arc(Vector2d center, double radius, double thetaStart, double thetaEnd){
        path.add(new CircularArcPath(center, radius, thetaStart, thetaEnd));
        return this;
    }
    public TrajectoryBuilder bezier(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3){
        path.add(new BezierPath(p0,p1,p2,p3));
        return this;
    }

    // --- build helpers ---

    /** Convert ds (inches) into a reasonable samples-per-segment for CompositePath.build(int). */
    private static int samplesPerSegmentFor(double ds){
        // smaller ds → more samples; clamp to sane range so we don’t explode CPU
        double dsClamped = Math.max(0.25, Math.min(2.0, ds)); // 0.25"…2" typical
        int sps = (int)Math.round(200.0 / dsClamped);         // ds=0.5 → ~400, ds=1.0 → ~200
        if (sps < MIN_SPS) sps = MIN_SPS;
        if (sps > MAX_SPS) sps = MAX_SPS;
        return sps;
    }

    /** Build using path tangent as heading (convenience: default ds). */
    public Trajectory buildTangentHeading(TrajectoryConstraints c){
        return buildTangentHeading(c, DEFAULT_DS, null);
    }

    /** Build using a custom heading profile (convenience: default ds). */
    public Trajectory buildWithHeading(TrajectoryConstraints c, HeadingProfile headingProfile){
        return buildWithHeading(c, DEFAULT_DS, headingProfile, null);
    }

    /** Build with heading from path tangent. */
    public Trajectory buildTangentHeading(TrajectoryConstraints c, double ds, Double headingSeedRad){
        int sps = samplesPerSegmentFor(ds);
        path.build(sps);

        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, c, ds, null, headingSeedRad);

        if (samples.isEmpty())
            throw new IllegalStateException("TimeParameterizer returned no samples.");

        List<Trajectory.State> out = new ArrayList<>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a, ts.pose,
                    ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }

    /** Build with a custom heading profile along arc length s (e.g., face a goal). */
    public Trajectory buildWithHeading(TrajectoryConstraints c, double ds, HeadingProfile headingProfile, Double headingSeedRad){
        if (headingProfile == null)
            throw new IllegalArgumentException("headingProfile cannot be null. Use buildTangentHeading for tangent heading.");

        int sps = samplesPerSegmentFor(ds);
        path.build(sps);

        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, c, ds, headingProfile, headingSeedRad);

        if (samples.isEmpty())
            throw new IllegalStateException("TimeParameterizer returned no samples.");

        List<Trajectory.State> out = new ArrayList<>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a, ts.pose,
                    ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }
}