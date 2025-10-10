package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.List;

/** Final time-parameterized trajectory (monotonic in t). */
public class Trajectory {
    public static class State {
        public final double t, s, v, a;    // time, arclength, along-track vel/accel
        public final Pose2d pose;          // x,y,heading
        public final double curvature;     // path curvature (1/in)
        public final double omega;         // heading rate (rad/s)
        public final double alpha;         // heading accel (rad/s^2)

        public State(double t,double s,double v,double a,
                     Pose2d pose,double curvature,double omega,double alpha){
            this.t=t; this.s=s; this.v=v; this.a=a;
            this.pose=pose; this.curvature=curvature; this.omega=omega; this.alpha=alpha;
        }
    }

    private final List<State> states;
    private final double duration;
    private final double lengthS;

    public Trajectory(List<State> states){
        if (states == null || states.isEmpty())
            throw new IllegalArgumentException("Trajectory states cannot be null/empty.");

        // basic monotonicity/sanity check (guards weird inputs)
        double lastT = -Double.MAX_VALUE;
        double lastS = -Double.MAX_VALUE;
        for (State st : states){
            if (!Double.isFinite(st.t) || !Double.isFinite(st.s))
                throw new IllegalArgumentException("Non-finite t/s in trajectory.");
            if (st.t < lastT - 1e-9)
                throw new IllegalArgumentException("Trajectory time must be nondecreasing.");
            if (st.s < lastS - 1e-6)
                throw new IllegalArgumentException("Arc length s must be nondecreasing.");
            lastT = st.t;
            lastS = st.s;
        }

        this.states = states;
        this.duration = states.get(states.size()-1).t;
        this.lengthS = states.get(states.size()-1).s;
    }

    /** Total planned time. */
    public double duration(){ return duration; }

    /** Total planned arc length (in). */
    public double lengthS(){ return lengthS; }

    /** First and last states (exact samples). */
    public State startState(){ return states.get(0); }
    public State endState(){ return states.get(states.size()-1); }

    /** (Optional) expose underlying states for debug/plotting. */
    public List<State> allStates(){ return states; }

    /** Sample by time with linear interpolation and shortest-angle heading lerp. */
    public State sample(double t){
        if (t <= 0) return states.get(0);
        if (t >= duration) return states.get(states.size()-1);

        int lo = 0, hi = states.size()-1;
        while (hi - lo > 1){
            int mid = (lo + hi) >>> 1;
            if (states.get(mid).t < t) lo = mid; else hi = mid;
        }

        State a = states.get(lo), b = states.get(hi);
        double denom = Math.max(1e-9, b.t - a.t);
        double u = (t - a.t) / denom;

        double x = a.pose.x + (b.pose.x - a.pose.x)*u;
        double y = a.pose.y + (b.pose.y - a.pose.y)*u;
        double h = lerpAngle(a.pose.heading, b.pose.heading, u);

        return new State(
                t,
                a.s + (b.s - a.s)*u,
                a.v + (b.v - a.v)*u,
                a.a + (b.a - a.a)*u,
                new Pose2d(x, y, h),
                a.curvature + (b.curvature - a.curvature)*u,
                a.omega + (b.omega - a.omega)*u,
                a.alpha + (b.alpha - a.alpha)*u
        );
    }

    // --- helpers ---
    private static double lerpAngle(double a,double b,double u){
        double da = normalize(b) - normalize(a);
        da = normalize(da);
        return normalize(a + da*u);
    }

    public double closestTimeTo(Pose2d p){
        if (states.isEmpty()) return 0.0;
        double bestDist2 = Double.POSITIVE_INFINITY;
        double bestT = 0.0;

        // linear scan is fine (your trajectories are short); optimize later if needed
        for (State s : states){
            double dx = s.pose.x - p.x;
            double dy = s.pose.y - p.y;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestDist2){
                bestDist2 = d2;
                bestT = s.t;
            }
        }
        return bestT;
    }

    private static double normalize(double x){
        while (x <= -Math.PI) x += 2*Math.PI;
        while (x >   Math.PI) x -= 2*Math.PI;
        return x;
    }
}