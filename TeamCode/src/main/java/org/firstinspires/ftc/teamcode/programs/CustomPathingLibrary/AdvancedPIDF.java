package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class AdvancedPIDF {
    private double kp, ki, kd, kf;
    private double integral = 0.0;
    private double prevMeas = 0.0;
    private boolean first = true;
    private double outMin = -1e9, outMax = 1e9;
    private double iLimit = 1e9;

    // derivative low-pass
    private double dFilt = 0.0;
    private double alpha = 1.0; // 1 -> no filter

    public AdvancedPIDF(double kp,double ki,double kd,double kf){
        this.kp=kp; this.ki=ki; this.kd=kd; this.kf=kf;
    }
    public void setOutputLimits(double min,double max){ outMin=min; outMax=max; }
    public void setIntegralLimit(double lim){ iLimit = Math.abs(lim); }
    public void setDerivativeFilter(double cutoffHz, double loopHz){
        if (cutoffHz <= 0 || loopHz <= 0) { alpha = 1.0; return; }
        double dt = 1.0/loopHz;
        double rc = 1.0/(2.0*Math.PI*cutoffHz);
        alpha = dt/(rc+dt);
    }
    public void reset(){ integral=0.0; first=true; dFilt=0.0; }

    /** setpoint is desired error=0; we provide (sp, meas) to keep a standard interface */
    public double update(double sp, double meas, double ff, double dt){
        double err = sp - meas;
        integral += err * dt;
        if (integral > iLimit) integral = iLimit;
        if (integral < -iLimit) integral = -iLimit;

        double dMeas = 0.0;
        if (!first && dt>1e-6) dMeas = (meas - prevMeas)/dt;
        first = false;
        prevMeas = meas;

        dFilt = dFilt + alpha * (dMeas - dFilt);

        double out = kp*err + ki*integral - kd*dFilt + kf*ff;
        if (out > outMax) out = outMax;
        if (out < outMin) out = outMin;
        return out;
    }
}
