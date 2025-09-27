package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** Build a CompositePath from primitives, time-parameterize with S-curve, and inject heading profile. */
public class TrajectoryBuilder {
    private final CompositePath path = new CompositePath();

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

    /** Build with heading from path tangent. */
    public Trajectory buildTangentHeading(TrajectoryConstraints c, double ds){
        path.build(300); // dense sampling per segment
        List<TimeParameterizer.TrajSample> samples = TimeParameterizer.parameterize(path, c, ds, null);
        List<Trajectory.State> out = new ArrayList<Trajectory.State>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            // heading already from tangent in ts.pose
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a, ts.pose, ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }

    /** Build with custom heading profile along s (e.g., face target). */
    public Trajectory buildWithHeading(TrajectoryConstraints c, double ds, HeadingProfile headingProfile){
        path.build(300);
        List<TimeParameterizer.TrajSample> samples = TimeParameterizer.parameterize(path, c, ds, headingProfile);
        List<Trajectory.State> out = new ArrayList<Trajectory.State>(samples.size());
        for (TimeParameterizer.TrajSample ts : samples){
            out.add(new Trajectory.State(ts.t, ts.s, ts.v, ts.a, ts.pose, ts.curvature, ts.omega, ts.alpha));
        }
        return new Trajectory(out);
    }
}
