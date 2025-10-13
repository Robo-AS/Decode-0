package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.List;

/** Final time-parameterized trajectory (samples over time). */
public class Trajectory {
    public static class State {
        public final double t, s, v, a;
        public final Pose2d pose;         // includes heading profile
        public final Vector2d tangent;    // *** unit path tangent (world) ***
        public final double curvature;    // kappa
        public final double omega;        // heading rate (rad/s)
        public final double alpha;        // heading accel (rad/s^2)

        public State(double t,double s,double v,double a,
                     Pose2d pose, Vector2d tangent,
                     double curvature, double omega, double alpha){
            this.t=t; this.s=s; this.v=v; this.a=a;
            this.pose=pose;
            this.tangent = (tangent==null) ? new Vector2d(1,0) : tangent;
            this.curvature=curvature; this.omega=omega; this.alpha=alpha;
        }
    }

    private final List<State> states;

    public Trajectory(List<State> states){
        if (states == null || states.isEmpty())
            throw new IllegalArgumentException("Trajectory states cannot be null/empty.");
        this.states = states;
        // (Optional) monotonicity checks can be added here if you want
    }

    public double duration(){ return states.get(states.size()-1).t; }
    public List<State> allStates(){ return states; }

    /** Sample by time with linear interp (shortest-angle for heading, renormalized tangent). */
    public State sample(double t){
        if (t <= 0) return states.get(0);
        if (t >= duration()) return states.get(states.size()-1);

        int lo=0, hi=states.size()-1;
        while (hi - lo > 1){
            int mid = (lo+hi)>>>1;
            if (states.get(mid).t < t) lo=mid; else hi=mid;
        }
        State a = states.get(lo), b = states.get(hi);
        double u = (t - a.t)/Math.max(1e-9, b.t - a.t);

        double x = a.pose.x + (b.pose.x - a.pose.x)*u;
        double y = a.pose.y + (b.pose.y - a.pose.y)*u;
        double h = lerpAngle(a.pose.heading, b.pose.heading, u);

        Vector2d tan = new Vector2d(
                a.tangent.x + (b.tangent.x - a.tangent.x)*u,
                a.tangent.y + (b.tangent.y - a.tangent.y)*u
        ).normalized();
        if (tan.norm() < 1e-6) tan = a.tangent;

        return new State(
                t,
                a.s + (b.s - a.s)*u,
                a.v + (b.v - a.v)*u,
                a.a + (b.a - a.a)*u,
                new Pose2d(x,y,h),
                tan,
                a.curvature + (b.curvature - a.curvature)*u,
                a.omega + (b.omega - a.omega)*u,
                a.alpha + (b.alpha - a.alpha)*u
        );
    }

    private static double lerpAngle(double a,double b,double u){
        double da = normalize(b) - normalize(a);
        da = normalize(da);
        return normalize(a + da*u);
    }
    private static double normalize(double x){
        while (x<=-Math.PI) x+=2*Math.PI;
        while (x> Math.PI)  x-=2*Math.PI;
        return x;
    }

    /** Return the time t of the sample whose (x,y) is closest to the given pose. */
    public double closestTimeTo(Pose2d p){
        if (states.isEmpty()) return 0.0;
        double bestD2 = Double.POSITIVE_INFINITY;
        double bestT  = states.get(0).t;

        for (State s : states){
            double dx = s.pose.x - p.x;
            double dy = s.pose.y - p.y;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestD2){
                bestD2 = d2;
                bestT  = s.t;
            }
        }
        return bestT;
    }
}