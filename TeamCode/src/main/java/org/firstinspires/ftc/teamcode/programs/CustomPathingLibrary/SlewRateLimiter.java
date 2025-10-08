package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class SlewRateLimiter {
    private final double maxRate;
    private double last;
    private boolean first=true;
    private static final double EPS = 1e-9;

    public SlewRateLimiter(double maxRatePerSec){
        this.maxRate=Math.max(0.0, Math.abs(maxRatePerSec));
    }

    public double filter(double target, double dt){
        if (!Double.isFinite(target)) target = 0.0;
        if (!Double.isFinite(dt) || dt <= 1e-6) dt = 1e-3;

        if (first){
            first=false; last=target; return target;
        }

        double maxDelta = maxRate * dt;
        double delta = target - last;

        // small deadband to avoid chattering
        if (Math.abs(delta) < Math.min(1e-6, maxDelta*0.05)) return last;

        if (delta >  maxDelta) delta =  maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;

        last += delta;
        return last;
    }

    public void reset(double value){ last=value; first=false; }
}