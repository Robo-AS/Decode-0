package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class AdvancedPIDF {
    private double kp, ki, kd, kf;
    private double integral = 0.0;
    private double prevMeas = 0.0;
    private boolean first = true;
    private double outMin = -1e9, outMax = 1e9;
    private double iLimit = 1e9;

    // derivative low-pass (first-order filter)
    private double dFilt = 0.0;
    private double alpha = 1.0; // 1 -> no filter

    public AdvancedPIDF(double kp,double ki,double kd,double kf){
        this.kp=kp; this.ki=ki; this.kd=kd; this.kf=kf;
    }

    public void setGains(double kp,double ki,double kd,double kf){
        this.kp=kp; this.ki=ki; this.kd=kd; this.kf=kf;
    }

    public void setOutputLimits(double min,double max){
        if (max < min){ double tmp=min; min=max; max=tmp; }
        outMin=min; outMax=max;
    }

    public void setIntegralLimit(double lim){ iLimit = Math.max(0.0, Math.abs(lim)); }

    /** Set derivative LPF alpha from cutoff & loop frequency (same as before). */
    public void setDerivativeFilter(double cutoffHz, double loopHz){
        if (cutoffHz <= 0 || loopHz <= 0) { alpha = 1.0; return; }
        double dt = 1.0/loopHz;
        double rc = 1.0/(2.0*Math.PI*cutoffHz);
        alpha = dt/(rc+dt);
        if (!Double.isFinite(alpha) || alpha <= 0) alpha = 1.0;
    }

    public void reset(){
        integral=0.0; first=true; dFilt=0.0; prevMeas=0.0;
    }

    /** Standard PIDF form; we interpret (sp - meas) as error, ff is feedforward term (units of output). */
    public double update(double sp, double meas, double ff, double dt){
        if (!Double.isFinite(sp)) sp = 0;
        if (!Double.isFinite(meas)) meas = 0;
        if (!Double.isFinite(ff)) ff = 0;
        if (!Double.isFinite(dt) || dt <= 1e-6) dt = 1e-3;

        final double err = sp - meas;

        // --- Derivative on measurement (robust to step in setpoint) + LPF ---
        double dMeas = 0.0;
        if (!first) dMeas = (meas - prevMeas)/dt;
        first = false;
        prevMeas = meas;

        dFilt += alpha * (dMeas - dFilt);

        // --- Provisional output (without integral) to check saturation direction ---
        double pTerm = kp * err;
        double dTerm = -kd * dFilt;   // note the minus (derivative on measurement)
        double outNoI = pTerm + dTerm + kf * ff;

        // --- Conditional integration (anti-windup) ---
        boolean wouldSaturateHigh = (outNoI > outMax) && (err > 0);
        boolean wouldSaturateLow  = (outNoI < outMin) && (err < 0);
        if (!(wouldSaturateHigh || wouldSaturateLow)) {
            integral += err * dt;
            // clamp integral alone
            if (integral > iLimit) integral = iLimit;
            if (integral < -iLimit) integral = -iLimit;
        }

        double out = outNoI + ki * integral;

        // final clamp
        if (out > outMax) out = outMax;
        if (out < outMin) out = outMin;

        if (!Double.isFinite(out)) out = 0.0;
        return out;
    }

    // Optional getters for telemetry/tuning
    public double getIntegral(){ return integral; }
    public double getAlpha(){ return alpha; }
}