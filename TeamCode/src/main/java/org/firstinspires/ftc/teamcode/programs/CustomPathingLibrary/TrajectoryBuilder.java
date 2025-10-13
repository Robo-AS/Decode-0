package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilder {
    public static final double DEFAULT_DS = 0.5;
    private static final int MIN_SPS=50, MAX_SPS=600;

    private final CompositePath path = new CompositePath();

    public TrajectoryBuilder line(Vector2d a, Vector2d b){ path.add(new LinePath(a,b)); return this; }
    public TrajectoryBuilder arc(Vector2d c, double r, double th0, double th1){ path.add(new CircularArcPath(c,r,th0,th1)); return this; }
    public TrajectoryBuilder bezier(Vector2d p0,Vector2d p1,Vector2d p2,Vector2d p3){ path.add(new BezierPath(p0,p1,p2,p3)); return this; }

    private static int spsFor(double ds){
        double dsC = Math.max(0.25, Math.min(2.0, ds));
        int sps = (int)Math.round(200.0/dsC);
        if (sps < MIN_SPS) sps=MIN_SPS;
        if (sps > MAX_SPS) sps=MAX_SPS;
        return sps;
    }

    public Trajectory buildTangentHeading(TrajectoryConstraints c){ return buildTangentHeading(c, DEFAULT_DS, null); }
    public Trajectory buildWithHeading(TrajectoryConstraints c, HeadingProfile hp){ return buildWithHeading(c, DEFAULT_DS, hp, null); }

    public Trajectory buildTangentHeading(TrajectoryConstraints c, double ds, Double headingSeedRad){
        path.buildByDs(ds); // use actual ds
        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, c, ds, null, headingSeedRad);

        List<Trajectory.State> out = new ArrayList<>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a,
                    ts.pose, ts.tangent, ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }

    public Trajectory buildWithHeading(TrajectoryConstraints c, double ds, HeadingProfile hp, Double headingSeedRad){
        if (hp == null) throw new IllegalArgumentException("headingProfile is null; use buildTangentHeading instead.");
        path.buildByDs(ds);
        List<TimeParameterizer.TrajSample> samples =
                TimeParameterizer.parameterize(path, c, ds, hp, headingSeedRad);

        List<Trajectory.State> out = new ArrayList<>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a,
                    ts.pose, ts.tangent, ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }
}