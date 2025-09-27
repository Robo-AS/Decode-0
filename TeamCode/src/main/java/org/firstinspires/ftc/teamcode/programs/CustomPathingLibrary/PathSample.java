package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/** Path sample at arc-length s: position, tangent (unit), curvature kappa. */
public class PathSample {
    public final double s;
    public final Vector2d pos;
    public final Vector2d tangent;   // unit vector
    public final double curvature;   // 1/inches (signed)
    public PathSample(double s, Vector2d pos, Vector2d tangent, double curvature){
        this.s=s; this.pos=pos; this.tangent=tangent; this.curvature=curvature;
    }
}
