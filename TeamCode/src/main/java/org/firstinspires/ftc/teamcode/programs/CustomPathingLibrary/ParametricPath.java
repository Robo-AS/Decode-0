package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/** Parametric path p(u), u∈[0,1], with derivatives for curvature. */
public interface ParametricPath {
    Vector2d p(double u);     // position
    Vector2d dp(double u);    // first derivative wrt u
    Vector2d ddp(double u);   // second derivative wrt u
    double length();          // approximate total arc length (precomputed)
}
