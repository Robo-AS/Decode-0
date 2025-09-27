package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class SlewRateLimiter {
    private final double maxRate;
    private double last;
    private boolean first=true;
    public SlewRateLimiter(double maxRatePerSec){ this.maxRate=Math.abs(maxRatePerSec); }
    public double filter(double target, double dt){
        if (first){ first=false; last=target; return target; }
        double maxDelta = maxRate*dt;
        double delta = target - last;
        if (delta >  maxDelta) delta =  maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        last += delta;
        return last;
    }
    public void reset(double value){ last=value; first=false; }
}
