package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Slew rate limiter for smooth command transitions.
 *
 * Limits the rate of change of a value to prevent sudden jumps.
 * Useful for motor commands, velocity setpoints, etc.
 *
 * Features:
 * - Configurable maximum rate of change
 * - Handles both positive and negative transitions
 * - Reset capability for new trajectories
 * - Asymmetric rate support (different accel/decel rates)
 */
public class SlewRateLimiter {

    private final double maxRateUp;    // Max rate of increase per second
    private final double maxRateDown;  // Max rate of decrease per second
    private double lastValue;
    private boolean initialized = false;

    // Small threshold to prevent floating-point chatter
    private static final double EPSILON = 1e-9;

    /**
     * Create a symmetric slew rate limiter.
     *
     * @param maxRatePerSec  Maximum rate of change per second (same for up and down)
     */
    public SlewRateLimiter(double maxRatePerSec) {
        this(maxRatePerSec, maxRatePerSec);
    }

    /**
     * Create an asymmetric slew rate limiter.
     *
     * @param maxRateUp    Maximum rate of increase per second
     * @param maxRateDown  Maximum rate of decrease per second
     */
    public SlewRateLimiter(double maxRateUp, double maxRateDown) {
        this.maxRateUp = Math.max(EPSILON, Math.abs(maxRateUp));
        this.maxRateDown = Math.max(EPSILON, Math.abs(maxRateDown));
        this.lastValue = 0.0;
        this.initialized = false;
    }

    /**
     * Filter the target value through the slew limiter.
     *
     * @param target  Desired value
     * @param dt      Time step in seconds
     * @return        Rate-limited value
     */
    public double filter(double target, double dt) {
        // Sanitize inputs
        if (!Double.isFinite(target)) {
            target = 0.0;
        }
        if (!Double.isFinite(dt) || dt <= EPSILON) {
            dt = 0.02;  // Default 50Hz
        }

        // Initialize on first call
        if (!initialized) {
            initialized = true;
            lastValue = target;
            return target;
        }

        // Calculate allowed change
        double delta = target - lastValue;

        // Determine which rate to use
        double maxDelta;
        if (delta > 0) {
            maxDelta = maxRateUp * dt;
        } else {
            maxDelta = maxRateDown * dt;
        }

        // Apply deadband to prevent tiny oscillations
        if (Math.abs(delta) < EPSILON) {
            return lastValue;
        }

        // Clamp the change
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }

        // Update and return
        lastValue += delta;

        // Final sanitization
        if (!Double.isFinite(lastValue)) {
            lastValue = 0.0;
        }

        return lastValue;
    }

    /**
     * Reset the limiter to a specific value.
     * Use when starting a new trajectory or after discontinuities.
     *
     * @param value  Value to reset to
     */
    public void reset(double value) {
        lastValue = Double.isFinite(value) ? value : 0.0;
        initialized = true;
    }

    /**
     * Reset the limiter to zero.
     */
    public void reset() {
        reset(0.0);
    }

    /**
     * Mark as uninitialized (next filter call will snap to target).
     */
    public void clear() {
        initialized = false;
        lastValue = 0.0;
    }

    /**
     * Get the current (last filtered) value.
     */
    public double getValue() {
        return lastValue;
    }

    /**
     * Check if the limiter has been initialized.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Check if the output has reached the target (within tolerance).
     */
    public boolean atTarget(double target, double tolerance) {
        return Math.abs(lastValue - target) <= tolerance;
    }

    /**
     * Get the maximum upward rate.
     */
    public double getMaxRateUp() {
        return maxRateUp;
    }

    /**
     * Get the maximum downward rate.
     */
    public double getMaxRateDown() {
        return maxRateDown;
    }

    /**
     * Calculate time to reach target from current value.
     */
    public double timeToTarget(double target) {
        double delta = target - lastValue;
        if (Math.abs(delta) < EPSILON) {
            return 0.0;
        }

        double rate = (delta > 0) ? maxRateUp : maxRateDown;
        return Math.abs(delta) / rate;
    }
}