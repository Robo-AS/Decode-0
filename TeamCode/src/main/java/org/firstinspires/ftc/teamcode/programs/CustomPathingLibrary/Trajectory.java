package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.List;

/** Final time-parameterized trajectory (samples over time). */
public class Trajectory {
    public static class State {
        public final double t, s, v, a;
        public final Pose2d pose;     // includes heading profile
        public final double curvature;
        public final double omega; //angular velocity (rad/s)
        public final double alpha; //angular acceleration(rad/s2)
        public State(double t,double s,double v,double a, Pose2d pose,double curvature, double omega, double alpha){
            this.t=t; this.s=s; this.v=v; this.a=a; this.pose=pose; this.curvature=curvature; this.omega=omega; this.alpha=alpha;
        }
    }

    private final List<State> states;

    public Trajectory(List<State> states){ this.states = states; }

    public double duration(){ return states.isEmpty()?0.0: states.get(states.size()-1).t; }

    /** Sample by time (nearest-neighbor for Java-8 simplicity). */
    public State sample(double t){
        if (states.isEmpty()) return new State(0,0,0,0,new Pose2d(0,0,0),0,0,0);
        if (t <= 0) return states.get(0);
        if (t >= duration()) return states.get(states.size()-1);
        int lo=0, hi=states.size()-1;
        while (hi - lo > 1){
            int mid = (lo+hi)/2;
            if (states.get(mid).t < t) lo=mid; else hi=mid;
        }
        // simple linear interp
        State a = states.get(lo), b = states.get(hi);
        double u = (t - a.t) / Math.max(1e-9, b.t - a.t);
        double x = a.pose.x + (b.pose.x - a.pose.x)*u;
        double y = a.pose.y + (b.pose.y - a.pose.y)*u;
        double h = lerpAngle(a.pose.heading, b.pose.heading, u);
        return new State(t,
                a.s + (b.s - a.s)*u,
                a.v + (b.v - a.v)*u,
                a.a + (b.a - a.a)*u,
                new Pose2d(x,y,h),
                a.curvature + (b.curvature - a.curvature)*u,
                a.omega + (b.omega - a.omega)*u,
                a.alpha + (b.alpha - a.alpha)*u);
    }

    public java.util.List<State> allStates(){ return states; }

    private static double lerpAngle(double a,double b,double u){
        double da = normalize(b) - normalize(a);
        da = normalize(da);
        return normalize(a + da*u);
    }
    private static double normalize(double x){
        while (x<=-Math.PI) x+=2*Math.PI;
        while (x> Math.PI) x-=2*Math.PI;
        return x;
    }
}
