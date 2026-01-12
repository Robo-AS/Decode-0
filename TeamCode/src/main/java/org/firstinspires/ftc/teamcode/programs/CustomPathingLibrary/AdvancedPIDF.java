package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Advanced PIDF Controller with:
 * - Derivative-on-measurement (prevents setpoint kick)
 * - Configurable low-pass filter on derivative
 * - Conditional integration (anti-windup)
 * - Output clamping
 * - Integral zone (only integrate when close to target)
 * - Error deadband to prevent oscillation
 * - Gain scheduling support
 */
public class AdvancedPIDF {
    // Gains
    private double kp, ki, kd, kf;

    // State
    private double integral = 0.0;
    private double prevMeas = 0.0;
    private double prevError = 0.0;
    private boolean first = true;

    // Output limits
    private double outMin = -1e9, outMax = 1e9;

    // Integral anti-windup
    private double iLimit = 1e9;
    private double iZone = Double.POSITIVE_INFINITY; // Only integrate when |error| < iZone

    // Derivative low-pass filter (first-order IIR)
    private double dFilt = 0.0;
    private double alpha = 1.0; // 1.0 = no filtering, lower = more smoothing

    // Error deadband (ignore tiny errors to prevent jitter)
    private double errorDeadband = 0.0;

    // For telemetry/debugging
    private double lastP = 0, lastI = 0, lastD = 0, lastF = 0, lastOutput = 0;

    public AdvancedPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    // ==================== CONFIGURATION ====================

    public void setGains(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public void setP(double kp) { this.kp = kp; }
    public void setI(double ki) { this.ki = ki; }
    public void setD(double kd) { this.kd = kd; }
    public void setF(double kf) { this.kf = kf; }

    public void setOutputLimits(double min, double max) {
        if (max < min) {
            double tmp = min;
            min = max;
            max = tmp;
        }
        outMin = min;
        outMax = max;
    }

    public void setIntegralLimit(double lim) {
        iLimit = Math.max(0.0, Math.abs(lim));
    }

    /**
     * Set integral zone - only accumulate integral when |error| < zone.
     * This prevents integral buildup when far from target.
     */
    public void setIntegralZone(double zone) {
        this.iZone = Math.max(0.0, Math.abs(zone));
    }

    /**
     * Set error deadband - errors smaller than this are treated as zero.
     * Helps prevent oscillation around the setpoint.
     */
    public void setErrorDeadband(double deadband) {
        this.errorDeadband = Math.max(0.0, Math.abs(deadband));
    }

    /**
     * Configure derivative low-pass filter from cutoff frequency and loop rate.
     * Lower cutoff = more smoothing (less noise, more lag).
     * Typical values: cutoff 10-30 Hz, loop 50-100 Hz.
     */
    public void setDerivativeFilter(double cutoffHz, double loopHz) {
        if (cutoffHz <= 0 || loopHz <= 0) {
            alpha = 1.0;
            return;
        }
        double dt = 1.0 / loopHz;
        double rc = 1.0 / (2.0 * Math.PI * cutoffHz);
        alpha = dt / (rc + dt);
        if (!Double.isFinite(alpha) || alpha <= 0 || alpha > 1.0) {
            alpha = 1.0;
        }
    }

    /**
     * Set derivative filter alpha directly (0 = max smoothing, 1 = no filter).
     */
    public void setDerivativeAlpha(double alpha) {
        this.alpha = Math.max(0.01, Math.min(1.0, alpha));
    }

    public void reset() {
        integral = 0.0;
        first = true;
        dFilt = 0.0;
        prevMeas = 0.0;
        prevError = 0.0;
        lastP = lastI = lastD = lastF = lastOutput = 0;
    }

    // ==================== MAIN UPDATE ====================

    /**
     * Standard PIDF update.
     *
     * @param setpoint  Desired value
     * @param measurement  Current measured value
     * @param feedforward  Feedforward term (units of output, e.g., velocity for position control)
     * @param dt  Time step in seconds
     * @return  Control output
     */
    public double update(double setpoint, double measurement, double feedforward, double dt) {
        // Sanitize inputs
        if (!Double.isFinite(setpoint)) setpoint = 0;
        if (!Double.isFinite(measurement)) measurement = 0;
        if (!Double.isFinite(feedforward)) feedforward = 0;
        if (!Double.isFinite(dt) || dt <= 1e-6) dt = 0.02; // Default 50Hz

        // Calculate error
        double error = setpoint - measurement;

        // Apply deadband
        if (Math.abs(error) < errorDeadband) {
            error = 0;
        }

        // --- Derivative on measurement (robust to setpoint steps) ---
        double dMeas = 0.0;
        if (!first) {
            dMeas = (measurement - prevMeas) / dt;
        }
        first = false;
        prevMeas = measurement;

        // Low-pass filter the derivative
        dFilt += alpha * (dMeas - dFilt);

        // --- Proportional term ---
        double pTerm = kp * error;

        // --- Derivative term (negative because derivative-on-measurement) ---
        double dTerm = -kd * dFilt;

        // --- Feedforward term ---
        double fTerm = kf * feedforward;

        // --- Calculate output without integral to check saturation ---
        double outNoI = pTerm + dTerm + fTerm;

        // --- Conditional integration (anti-windup) ---
        // Only integrate if:
        // 1. Error is within integral zone
        // 2. Output is not saturated in the direction that would increase integral
        boolean inIZone = Math.abs(error) < iZone;
        boolean wouldSaturateHigh = (outNoI >= outMax) && (error > 0);
        boolean wouldSaturateLow = (outNoI <= outMin) && (error < 0);

        if (inIZone && !(wouldSaturateHigh || wouldSaturateLow)) {
            integral += error * dt;
            // Clamp integral
            integral = Math.max(-iLimit, Math.min(iLimit, integral));
        }

        // If error crosses zero, optionally reset integral to prevent overshoot
        if (prevError != 0 && Math.signum(error) != Math.signum(prevError)) {
            integral *= 0.5; // Reduce integral on zero-crossing
        }
        prevError = error;

        double iTerm = ki * integral;

        // --- Final output ---
        double output = pTerm + iTerm + dTerm + fTerm;

        // Clamp output
        output = Math.max(outMin, Math.min(outMax, output));

        // Sanitize
        if (!Double.isFinite(output)) output = 0.0;

        // Store for telemetry
        lastP = pTerm;
        lastI = iTerm;
        lastD = dTerm;
        lastF = fTerm;
        lastOutput = output;

        return output;
    }

    /**
     * Simplified update without feedforward.
     */
    public double update(double setpoint, double measurement, double dt) {
        return update(setpoint, measurement, 0.0, dt);
    }

    // ==================== TELEMETRY GETTERS ====================

    public double getIntegral() { return integral; }
    public double getAlpha() { return alpha; }
    public double getLastP() { return lastP; }
    public double getLastI() { return lastI; }
    public double getLastD() { return lastD; }
    public double getLastF() { return lastF; }
    public double getLastOutput() { return lastOutput; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getKf() { return kf; }
}