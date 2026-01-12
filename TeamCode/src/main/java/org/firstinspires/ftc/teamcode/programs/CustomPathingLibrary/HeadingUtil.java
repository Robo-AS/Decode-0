package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/**
 * Utility functions for heading/angle manipulation.
 *
 * All angles are in radians unless otherwise specified.
 * Convention: positive = counter-clockwise (CCW), negative = clockwise (CW)
 */
public final class HeadingUtil {

    private HeadingUtil() {} // No instantiation

    // ==================== WRAPPING ====================

    /**
     * Wrap angle to (-π, π].
     *
     * @param angle  Input angle in radians
     * @return Wrapped angle in (-π, π]
     */
    public static double wrap(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    /**
     * Wrap angle to [0, 2π).
     */
    public static double wrap0To2Pi(double angle) {
        while (angle < 0) angle += 2.0 * Math.PI;
        while (angle >= 2.0 * Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    /**
     * Wrap angle in degrees to (-180, 180].
     */
    public static double wrapDegrees(double angleDeg) {
        while (angleDeg <= -180) angleDeg += 360;
        while (angleDeg > 180) angleDeg -= 360;
        return angleDeg;
    }

    // ==================== DELTAS ====================

    /**
     * Calculate the shortest signed angle from 'from' to 'to'.
     * Result is in (-π, π].
     *
     * Positive result means turn CCW, negative means turn CW.
     */
    public static double shortestDelta(double from, double to) {
        return wrap(wrap(to) - wrap(from));
    }

    /**
     * Calculate absolute angle difference (always positive).
     */
    public static double absoluteDelta(double from, double to) {
        return Math.abs(shortestDelta(from, to));
    }

    /**
     * Check if two angles are within a tolerance of each other.
     */
    public static boolean anglesEqual(double a, double b, double toleranceRad) {
        return absoluteDelta(a, b) <= toleranceRad;
    }

    // ==================== INTERPOLATION ====================

    /**
     * Linear interpolation between angles using shortest path.
     *
     * @param from  Starting angle
     * @param to    Ending angle
     * @param t     Interpolation factor [0, 1]
     * @return Interpolated angle
     */
    public static double lerp(double from, double to, double t) {
        double delta = shortestDelta(from, to);
        return wrap(from + delta * t);
    }

    /**
     * Smoothstep interpolation (ease-in-ease-out).
     * Uses 3t² - 2t³ curve.
     */
    public static double smoothLerp(double from, double to, double t) {
        t = smooth01(t);
        return lerp(from, to, t);
    }

    // ==================== UNWRAPPING ====================

    /**
     * Unwrap target angle to be continuous with reference.
     * Adds/subtracts 2π to target to minimize distance from reference.
     *
     * Useful for creating continuous heading sequences without wrap jumps.
     *
     * @param target     The angle to unwrap
     * @param reference  The reference angle (may already be unwrapped/continuous)
     * @return target adjusted to be within π of reference
     */
    public static double unwrapToNear(double target, double reference) {
        double t = wrap(target);
        double delta = t - wrap(reference);
        delta = wrap(delta);
        return reference + delta;
    }

    /**
     * Unwrap an array of angles to be continuous.
     * Modifies the array in place.
     */
    public static void unwrapArray(double[] angles) {
        if (angles == null || angles.length < 2) return;

        for (int i = 1; i < angles.length; i++) {
            angles[i] = unwrapToNear(angles[i], angles[i - 1]);
        }
    }

    // ==================== CLAMPING ====================

    /**
     * Clamp a value to [0, 1].
     */
    public static double clamp01(double t) {
        if (t < 0) return 0;
        if (t > 1) return 1;
        return t;
    }

    /**
     * Clamp a value to [min, max].
     */
    public static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    // ==================== SMOOTHING ====================

    /**
     * Smoothstep function: 3t² - 2t³
     * Maps [0, 1] → [0, 1] with zero derivatives at endpoints.
     */
    public static double smooth01(double t) {
        t = clamp01(t);
        return t * t * (3.0 - 2.0 * t);
    }

    /**
     * Smoother step function: 6t⁵ - 15t⁴ + 10t³
     * Even smoother transition with zero first AND second derivatives at endpoints.
     */
    public static double smoother01(double t) {
        t = clamp01(t);
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    // ==================== CONVERSIONS ====================

    /**
     * Convert radians to degrees.
     */
    public static double toDegrees(double radians) {
        return radians * (180.0 / Math.PI);
    }

    /**
     * Convert degrees to radians.
     */
    public static double toRadians(double degrees) {
        return degrees * (Math.PI / 180.0);
    }

    // ==================== DIRECTION UTILITIES ====================

    /**
     * Get the angle pointing from point A to point B.
     */
    public static double angleTo(double fromX, double fromY, double toX, double toY) {
        return Math.atan2(toY - fromY, toX - fromX);
    }

    /**
     * Get the angle pointing from point A to point B.
     */
    public static double angleTo(Vector2d from, Vector2d to) {
        return angleTo(from.x, from.y, to.x, to.y);
    }

    /**
     * Check if an angle is within a sector defined by center angle and half-width.
     */
    public static boolean inSector(double angle, double sectorCenter, double halfWidth) {
        return absoluteDelta(angle, sectorCenter) <= halfWidth;
    }

    /**
     * Get the opposite direction (add π radians).
     */
    public static double opposite(double angle) {
        return wrap(angle + Math.PI);
    }

    /**
     * Get perpendicular direction (add π/2 radians, CCW).
     */
    public static double perpendicular(double angle) {
        return wrap(angle + Math.PI / 2.0);
    }

    /**
     * Get perpendicular direction clockwise (subtract π/2).
     */
    public static double perpendicularCW(double angle) {
        return wrap(angle - Math.PI / 2.0);
    }

    // ==================== ANGULAR VELOCITY ====================

    /**
     * Calculate angular velocity needed to change from current to target heading
     * in the given time.
     */
    public static double angularVelocityFor(double currentHeading, double targetHeading, double time) {
        if (time <= 0) return 0;
        double delta = shortestDelta(currentHeading, targetHeading);
        return delta / time;
    }

    /**
     * Determine turn direction (-1 = CW, +1 = CCW, 0 = no turn needed).
     */
    public static int turnDirection(double from, double to, double toleranceRad) {
        double delta = shortestDelta(from, to);
        if (Math.abs(delta) < toleranceRad) return 0;
        return (delta > 0) ? 1 : -1;
    }
}