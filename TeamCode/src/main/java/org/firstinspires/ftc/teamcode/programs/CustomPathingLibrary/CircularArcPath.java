package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;



/** 2D circular arc from A to B with given center and direction. */
public class CircularArcPath implements ParametricPath {
    private final Vector2d center;
    private final double radius;   // inches
    private final double theta0;   // start angle
    private final double dTheta;   // total sweep (signed)
    private final double len;

    public CircularArcPath(Vector2d center, double radius, double thetaStart, double thetaEnd){
        this.center = center;
        this.radius = Math.abs(radius);
        // choose shortest signed sweep from start to end
        double d = thetaEnd - thetaStart;
        while (d <= -Math.PI) d += 2*Math.PI;
        while (d >   Math.PI) d -= 2*Math.PI;
        this.theta0 = thetaStart;
        this.dTheta = d;
        this.len = Math.abs(dTheta) * this.radius;
    }

    public Vector2d p(double u){
        double th = theta0 + dTheta * u;
        return new Vector2d(center.x + radius*Math.cos(th), center.y + radius*Math.sin(th));
    }
    public Vector2d dp(double u){
        double th = theta0 + dTheta * u;
        double dth_du = dTheta;
        return new Vector2d(-radius*Math.sin(th)*dth_du, radius*Math.cos(th)*dth_du);
    }
    public Vector2d ddp(double u){
        double th = theta0 + dTheta * u;
        double dth = dTheta, dth2 = dth*dth;
        return new Vector2d(-radius*Math.cos(th)*dth2, -radius*Math.sin(th)*dth2);
    }
    public double length(){ return len; }
}
