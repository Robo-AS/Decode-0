package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

/**
 * goBILDA Pinpoint Odometry Computer localizer.
 *
 * Features:
 * - High-precision odometry from Pinpoint's onboard fusion
 * - Configurable pod offsets and encoder directions
 * - Velocity estimation
 * - External pose fusion (e.g., AprilTags/Limelight)
 * - Bulk read decimation for I2C optimization
 * - Robust error handling
 *
 * Outputs pose in INCHES (x, y) and RADIANS (heading).
 */
public class GoBildaPinpointLocalizer {

    // ==================== CONSTANTS ====================

    private static final double MM_PER_INCH = 25.4;

    // ==================== HARDWARE ====================

    private final String deviceName;
    private GoBildaPinpointDriver pinpoint;
    private boolean initialized = false;

    // ==================== POSE STATE ====================

    // Current pose (inches, radians)
    private final Pose2d pose = new Pose2d(0, 0, 0);

    // Velocities
    private double vxInPerSec = 0.0;   // Forward velocity (in/s)
    private double vyInPerSec = 0.0;   // Strafe velocity (in/s)
    private double omegaRadPerSec = 0.0; // Angular velocity (rad/s)

    // ==================== CONFIGURATION ====================

    // Encoder resolution
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderPreset =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private Double customTicksPerMM = null;

    // Pod offsets (mm) relative to robot center
    // +X = left of center, +Y = forward of center
    private double xOffsetMM = 0.0;
    private double yOffsetMM = 0.0;

    // Encoder directions
    private GoBildaPinpointDriver.EncoderDirection xDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection yDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // IMU yaw scalar (fine-tuning for gyro drift)
    private Double yawScalar = null;

    // Bulk read decimation
    private int bulkReadInterval = 1;  // 1 = every loop, 2 = every other loop, etc.
    private int loopCounter = 0;

    // ==================== CONSTRUCTOR ====================

    /**
     * Create localizer with device name.
     *
     * @param deviceName  Hardware name in robot configuration (e.g., "pinpoint")
     */
    public GoBildaPinpointLocalizer(String deviceName) {
        this.deviceName = deviceName;
    }

    // ==================== CONFIGURATION (FLUENT API) ====================

    /**
     * Set encoder resolution using a goBILDA preset.
     */
    public GoBildaPinpointLocalizer setEncoderResolutionPreset(
            GoBildaPinpointDriver.GoBildaOdometryPods preset) {
        this.encoderPreset = preset;
        this.customTicksPerMM = null;
        if (initialized && pinpoint != null) {
            pinpoint.setEncoderResolution(preset);
        }
        return this;
    }

    /**
     * Set custom encoder resolution (ticks per mm).
     * Overrides the preset.
     */
    public GoBildaPinpointLocalizer setEncoderResolutionTicksPerMM(double ticksPerMM) {
        this.customTicksPerMM = ticksPerMM;
        if (initialized && pinpoint != null) {
            pinpoint.setEncoderResolution(ticksPerMM);
        }
        return this;
    }

    /**
     * Set pod offsets in millimeters.
     *
     * @param xOffsetMM  X offset (positive = left of robot center)
     * @param yOffsetMM  Y offset (positive = forward of robot center)
     */
    public GoBildaPinpointLocalizer setPodOffsetsMM(double xOffsetMM, double yOffsetMM) {
        this.xOffsetMM = xOffsetMM;
        this.yOffsetMM = yOffsetMM;
        if (initialized && pinpoint != null) {
            pinpoint.setOffsets(xOffsetMM, yOffsetMM);
        }
        return this;
    }

    /**
     * Set pod offsets in inches (converted to mm internally).
     */
    public GoBildaPinpointLocalizer setPodOffsetsInches(double xOffsetIn, double yOffsetIn) {
        return setPodOffsetsMM(xOffsetIn * MM_PER_INCH, yOffsetIn * MM_PER_INCH);
    }

    /**
     * Set encoder counting directions.
     */
    public GoBildaPinpointLocalizer setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection xDir,
            GoBildaPinpointDriver.EncoderDirection yDir) {
        this.xDirection = xDir;
        this.yDirection = yDir;
        if (initialized && pinpoint != null) {
            pinpoint.setEncoderDirections(xDir, yDir);
        }
        return this;
    }

    /**
     * Set IMU yaw scalar for fine-tuning gyro accuracy.
     * Default is 1.0. Adjust if heading drifts consistently.
     */
    public GoBildaPinpointLocalizer setYawScalar(double scalar) {
        this.yawScalar = scalar;
        if (initialized && pinpoint != null) {
            pinpoint.setYawScalar(scalar);
        }
        return this;
    }

    /**
     * Set bulk read interval for I2C optimization.
     * Higher values reduce I2C traffic but lower update rate.
     *
     * @param interval  1 = every loop (default), 2 = every other loop, etc.
     */
    public GoBildaPinpointLocalizer setBulkReadInterval(int interval) {
        this.bulkReadInterval = Math.max(1, interval);
        return this;
    }

    // ==================== LIFECYCLE ====================

    /**
     * Initialize the localizer. Call once in init().
     */
    public void init(HardwareMap hardwareMap) {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);

            // Apply configuration
            if (customTicksPerMM != null) {
                pinpoint.setEncoderResolution(customTicksPerMM);
            } else {
                pinpoint.setEncoderResolution(encoderPreset);
            }

            pinpoint.setOffsets(xOffsetMM, yOffsetMM);
            pinpoint.setEncoderDirections(xDirection, yDirection);

            if (yawScalar != null) {
                pinpoint.setYawScalar(yawScalar);
            }

            initialized = true;
        } catch (Exception e) {
            initialized = false;
            System.err.println("GoBildaPinpointLocalizer: Failed to initialize - " + e.getMessage());
        }
    }

    /**
     * Reset position and recalibrate IMU.
     * Robot must be stationary when calling this!
     */
    public void resetPosAndIMU() {
        if (initialized && pinpoint != null) {
            pinpoint.resetPosAndIMU();
            pose.x = 0;
            pose.y = 0;
            pose.heading = 0;
            vxInPerSec = 0;
            vyInPerSec = 0;
            omegaRadPerSec = 0;
        }
    }

    /**
     * Recalibrate IMU only (keeps position).
     * Robot must be stationary!
     */
    public void recalibrateIMU() {
        if (initialized && pinpoint != null) {
            pinpoint.recalibrateIMU();
        }
    }

    /**
     * Set the current pose explicitly.
     * Useful for setting starting position or correcting drift.
     */
    public void setPose(Pose2d newPose) {
        if (initialized && pinpoint != null) {
            Pose2D p2d = new Pose2D(DistanceUnit.INCH, newPose.x, newPose.y,
                    AngleUnit.RADIANS, newPose.heading);
            pinpoint.setPosition(p2d);
        }
        pose.x = newPose.x;
        pose.y = newPose.y;
        pose.heading = wrapAngle(newPose.heading);
    }

    /**
     * Set pose from components.
     */
    public void setPose(double x, double y, double headingRad) {
        setPose(new Pose2d(x, y, headingRad));
    }

    // ==================== RUNTIME UPDATE ====================

    /**
     * Update the localizer. Call every loop iteration.
     *
     * @param dt  Time since last update (seconds) - not used by Pinpoint but kept for interface
     */
    public void update(double dt) {
        if (!initialized || pinpoint == null) {
            return;
        }

        loopCounter++;
        boolean doBulkRead = (loopCounter % bulkReadInterval == 0);

        try {
            if (doBulkRead) {
                // Full update: position + velocity + heading
                pinpoint.update();

                // Read position (mm -> inches)
                pose.x = pinpoint.getPosX() / MM_PER_INCH;
                pose.y = pinpoint.getPosY() / MM_PER_INCH;
                pose.heading = wrapAngle(pinpoint.getHeading());

                // Read velocities (mm/s -> in/s)
                vxInPerSec = pinpoint.getVelX() / MM_PER_INCH;
                vyInPerSec = pinpoint.getVelY() / MM_PER_INCH;
                omegaRadPerSec = pinpoint.getHeadingVelocity();

            } else {
                // Lightweight update: heading only (reduces I2C traffic)
                pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
                pose.heading = wrapAngle(pinpoint.getHeading());
            }
        } catch (Exception e) {
            System.err.println("GoBildaPinpointLocalizer: Update error - " + e.getMessage());
        }
    }

    /**
     * Update without dt parameter.
     */
    public void update() {
        update(0.02);  // Assume 50Hz default
    }

    // ==================== POSE ACCESS ====================

    /**
     * Get current pose (inches, radians).
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Get X position (inches).
     */
    public double getX() {
        return pose.x;
    }

    /**
     * Get Y position (inches).
     */
    public double getY() {
        return pose.y;
    }

    /**
     * Get heading (radians, wrapped to (-π, π]).
     */
    public double getHeading() {
        return pose.heading;
    }

    /**
     * Get heading in degrees.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(pose.heading);
    }

    // ==================== VELOCITY ACCESS ====================

    /**
     * Get forward velocity (inches/second).
     */
    public double getVX() {
        return vxInPerSec;
    }

    /**
     * Get strafe velocity (inches/second).
     */
    public double getVY() {
        return vyInPerSec;
    }

    /**
     * Get angular velocity (radians/second).
     */
    public double getOmega() {
        return omegaRadPerSec;
    }

    /**
     * Get linear speed magnitude (inches/second).
     */
    public double getSpeed() {
        return Math.hypot(vxInPerSec, vyInPerSec);
    }

    // ==================== EXTERNAL POSE FUSION ====================

    /**
     * Fuse an external pose measurement (e.g., from AprilTags) with current pose.
     * Uses weighted averaging for smooth correction.
     *
     * @param externalPose  Pose from external source (inches, radians)
     * @param alpha         Blend factor [0, 1]: 0 = keep current, 1 = snap to external
     */
    public void fuseExternalPose(Pose2d externalPose, double alpha) {
        if (!initialized || pinpoint == null || externalPose == null) {
            return;
        }

        alpha = Math.max(0.0, Math.min(1.0, alpha));

        // Weighted average for position
        double newX = pose.x + alpha * (externalPose.x - pose.x);
        double newY = pose.y + alpha * (externalPose.y - pose.y);

        // Shortest-path interpolation for heading
        double headingDelta = wrapAngle(externalPose.heading - pose.heading);
        double newHeading = wrapAngle(pose.heading + alpha * headingDelta);

        // Apply fused pose to both internal state and device
        setPose(new Pose2d(newX, newY, newHeading));
    }

    /**
     * Fuse external pose with automatic alpha based on confidence.
     *
     * @param externalPose  Pose from external source
     * @param confidence    Confidence level [0, 1]
     * @param maxAlpha      Maximum blend factor to use
     */
    public void fuseExternalPose(Pose2d externalPose, double confidence, double maxAlpha) {
        double alpha = confidence * maxAlpha;
        fuseExternalPose(externalPose, alpha);
    }

    // ==================== STATUS ====================

    /**
     * Check if localizer is initialized and working.
     */
    public boolean isInitialized() {
        return initialized && pinpoint != null;
    }

    /**
     * Get device status string.
     */
    public String getStatusString() {
        if (!initialized || pinpoint == null) {
            return "NOT_INITIALIZED";
        }
        try {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            return (status != null) ? status.name() : "UNKNOWN";
        } catch (Exception e) {
            return "ERROR";
        }
    }

    /**
     * Get device update frequency (Hz).
     */
    public double getFrequency() {
        if (!initialized || pinpoint == null) return 0;
        try {
            return pinpoint.getFrequency();
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * Get loop time in microseconds.
     */
    public int getLoopTimeMicros() {
        if (!initialized || pinpoint == null) return 0;
        try {
            return pinpoint.getLoopTime();
        } catch (Exception e) {
            return 0;
        }
    }

    // ==================== UTILITY ====================

    /**
     * Wrap angle to (-π, π].
     */
    private static double wrapAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }
}