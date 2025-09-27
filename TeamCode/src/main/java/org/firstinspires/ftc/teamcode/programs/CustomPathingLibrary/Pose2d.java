package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class Pose2d {
    public double x, y, heading; // inches, radians
    public Pose2d(double x, double y, double heading){ this.x=x; this.y=y; this.heading=heading; }
    public Pose2d copy(){ return new Pose2d(x,y,heading); }
}
