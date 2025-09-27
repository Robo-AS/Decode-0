package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class Vector2d {
    public double x, y;
    public Vector2d(double x,double y){ this.x=x; this.y=y; }

    public double norm(){ return Math.hypot(x,y); }
    public Vector2d plus(Vector2d v){ return new Vector2d(x+v.x, y+v.y); }
    public Vector2d minus(Vector2d v){ return new Vector2d(x-v.x, y-v.y); }
    public Vector2d times(double s){ return new Vector2d(x*s, y*s); }
    public Vector2d normalized(){ double n=norm(); return (n>1e-9)? new Vector2d(x/n,y/n):new Vector2d(0,0); }
}
