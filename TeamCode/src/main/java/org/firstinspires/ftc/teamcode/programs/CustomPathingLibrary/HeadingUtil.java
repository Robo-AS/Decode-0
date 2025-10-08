package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

final class HeadingUtil {
    private HeadingUtil() {}

    /** Wrap to (-π, π]. */
    static double wrap(double a){
        while (a <= -Math.PI) a += 2.0*Math.PI;
        while (a >   Math.PI) a -= 2.0*Math.PI;
        return a;
    }

    /** Shortest signed delta from a→b, in (-π, π]. */
    static double shortestDelta(double a, double b){
        return wrap(wrap(b) - wrap(a));
    }

    /** Smoothstep S-curve: 3t^2 - 2t^3, t∈[0,1]. */
    static double smooth01(double t){
        if (t <= 0) return 0;
        if (t >= 1) return 1;
        return t*t*(3 - 2*t);
    }

    /** Clamp. */
    static double clamp01(double t){
        if (t < 0) return 0;
        if (t > 1) return 1;
        return t;
    }

    public static double unwrapToNear(double target, double reference) {
        double t = wrap(target);
        double r = reference; // assume r may already be “continuous”
        double d = t - wrap(r);
        d = wrap(d);
        return r+d;
    }
}