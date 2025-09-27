package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class LinePath implements ParametricPath {
    private final Vector2d a, b;
    private final Vector2d delta;
    private final double len;

    public LinePath(Vector2d a, Vector2d b){
        this.a=a; this.b=b;
        this.delta = b.minus(a);
        this.len = delta.norm();
    }

    public Vector2d p(double u){ return new Vector2d(a.x + delta.x*u, a.y + delta.y*u); }
    public Vector2d dp(double u){ return delta; }
    public Vector2d ddp(double u){ return new Vector2d(0,0); }
    public double length(){ return len; }
}
