package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/** Cubic Bezier with four control points. */
public class BezierPath implements ParametricPath {
    private final Vector2d p0,p1,p2,p3;
    private final double len;

    public BezierPath(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3){
        this.p0=p0; this.p1=p1; this.p2=p2; this.p3=p3;
        this.len = approximateLength(200);
    }

    private double approximateLength(int steps){
        double L=0; Vector2d prev = p(0);
        for (int i=1;i<=steps;i++){
            double u = (double)i/steps;
            Vector2d cur = p(u);
            L += cur.minus(prev).norm();
            prev = cur;
        }
        return L;
    }

    public Vector2d p(double u){
        double v = 1-u;
        double b0 = v*v*v;
        double b1 = 3*v*v*u;
        double b2 = 3*v*u*u;
        double b3 = u*u*u;
        return new Vector2d(
                b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x,
                b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y
        );
    }

    public Vector2d dp(double u){
        double v = 1-u;
        // derivative of cubic Bezier
        Vector2d a = p1.minus(p0).times(3*v*v);
        Vector2d b = p2.minus(p1).times(6*v*u);
        Vector2d c = p3.minus(p2).times(3*u*u);
        return new Vector2d(a.x+b.x+c.x, a.y+b.y+c.y);
    }

    public Vector2d ddp(double u){
        double v = 1-u;
        Vector2d a = p2.minus(p1.times(2)).plus(p0).times(6*v);
        Vector2d b = p3.minus(p2.times(2)).plus(p1).times(6*u);
        return new Vector2d(a.x+b.x, a.y+b.y);
    }

    public double length(){ return len; }
}
