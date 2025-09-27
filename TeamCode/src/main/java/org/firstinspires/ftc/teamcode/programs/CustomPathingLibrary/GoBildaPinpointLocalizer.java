package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

/**
 * goBILDA Pinpoint Odometry Computer localizer (I2C).
 * - Outputs pose in INCHES (x,y) and RADIANS (heading).
 * - Uses Pinpoint's onboard encoder+IMU fusion.
 * - Provides encoder preset/custom resolution, pod offsets, encoder directions.
 * - Provides reset/recalibrate IMU, yaw scalar tuning.
 * - Supports bulk-read decimation (heading-only between bulks) to reduce I2C load.
 * - Allows external absolute pose fusion (e.g., AprilTags/Limelight) via smooth blending.
 */
public class GoBildaPinpointLocalizer {

    // ---- constants & conversion ----
    private static final double MM_PER_IN = 25.4;

    // ---- hardware ----
    private final String hwName; // RC config name, e.g. "pinpoint"
    private GoBildaPinpointDriver pinpoint;

    // ---- current state (inches, radians) ----
    private final Pose2d poseIn = new Pose2d(0, 0, 0);
    private double vxIn = 0.0;          // inches / sec
    private double vyIn = 0.0;          // inches / sec
    private double omegaRad = 0.0;      // rad / sec

    // ---- config (can be set before or after init; setters apply immediately if initialized) ----
    private GoBildaPinpointDriver.GoBildaOdometryPods preset = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private Double customTicksPerMM = null;   // if set, overrides preset
    private double xOffsetMM = -84;           // pod X offset from robot center (mm). Left=+
    private double yOffsetMM = -168;          // pod Y offset from robot center (mm). Forward=+
    private GoBildaPinpointDriver.EncoderDirection dirX = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection dirY = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private Double yawScalar = null;          // optional fine scale for gyro

    // ---- bulk decimation (I2C load control) ----
    private int bulkEveryN = 1; // 1 = bulk every loop, 2 = bulk every other loop, etc.
    private int decimCounter = 0;

    public GoBildaPinpointLocalizer(String hardwareMapName){
        this.hwName = hardwareMapName;
    }

    // ===================== CONFIG API =====================

    /** Use one of the goBILDA presets for ticks/mm (default = 4-bar pod). */
    public GoBildaPinpointLocalizer setEncoderResolutionPreset(GoBildaPinpointDriver.GoBildaOdometryPods p){
        this.preset = p;
        if (pinpoint != null && customTicksPerMM == null) {
            pinpoint.setEncoderResolution(preset);
        }
        return this;
    }

    /** Override the preset with your own ticks-per-mm (encoderCPR / wheelCircumferenceMM). */
    public GoBildaPinpointLocalizer setEncoderResolutionTicksPerMM(double ticksPerMM){
        this.customTicksPerMM = ticksPerMM;
        if (pinpoint != null) {
            pinpoint.setEncoderResolution(ticksPerMM);
        }
        return this;
    }

    /** Pod offsets in mm (X left+, Y forward+) relative to robot center. */
    public GoBildaPinpointLocalizer setPodOffsetsMM(double xOffsetMM, double yOffsetMM){
        this.xOffsetMM = xOffsetMM;
        this.yOffsetMM = yOffsetMM;
        if (pinpoint != null) pinpoint.setOffsets(xOffsetMM, yOffsetMM);
        return this;
    }

    /** Encoder directions: X increases forward; Y increases when strafing left. */
    public GoBildaPinpointLocalizer setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xDir,
                                                         GoBildaPinpointDriver.EncoderDirection yDir){
        this.dirX = xDir; this.dirY = yDir;
        if (pinpoint != null) pinpoint.setEncoderDirections(dirX, dirY);
        return this;
    }

    /** Optional IMU scale factor tweak (usually ~1.0). */
    public GoBildaPinpointLocalizer setYawScalar(Double scalar){
        this.yawScalar = scalar;
        if (pinpoint != null && yawScalar != null) pinpoint.setYawScalar(yawScalar);
        return this;
    }

    /** Read full 40B bulk every N loops; heading-only in between (saves I2C). */
    public GoBildaPinpointLocalizer setBulkEveryN(int n){
        this.bulkEveryN = Math.max(1, n);
        return this;
    }

    // ===================== LIFECYCLE =====================

    /** Call once in init(). */
    public void init(HardwareMap hw){
        pinpoint = hw.get(GoBildaPinpointDriver.class, hwName);

        // resolution
        if (customTicksPerMM != null) pinpoint.setEncoderResolution(customTicksPerMM);
        else pinpoint.setEncoderResolution(preset);

        // offsets, directions
        pinpoint.setOffsets(xOffsetMM, yOffsetMM);
        pinpoint.setEncoderDirections(dirX, dirY);

        // optional yaw scalar
        if (yawScalar != null) pinpoint.setYawScalar(yawScalar);
    }

    /** Hard-set absolute pose on the device (inches + radians). */
    public void setPose(Pose2d p){
        Pose2D p2 = new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading);
        pinpoint.setPosition(p2); // device expects mm/rad internally; driver handles unit writes per registers
        poseIn.x = p.x; poseIn.y = p.y; poseIn.heading = wrap(p.heading);
    }

    /** Best practice in INIT: call while robot is perfectly still. Resets encoders & IMU bias. */
    public void resetPosAndIMU(){
        if (pinpoint != null) pinpoint.resetPosAndIMU();
    }

    /** Re-zero IMU bias without resetting position (robot must be still). */
    public void recalibrateIMU(){
        if (pinpoint != null) pinpoint.recalibrateIMU();
    }

    // ===================== RUNTIME =====================

    /**
     * Call every loop. If bulk decimation is enabled, performs full bulk read every N loops
     * and heading-only updates on in-between loops.
     */
    public void update(double dt){
        if (pinpoint == null) return;

        boolean doBulk = (bulkEveryN <= 1) || (decimCounter++ % bulkEveryN == 0);

        if (doBulk){
            pinpoint.update(); // pos + vel + heading
            // positions (mm → in)
            poseIn.x = pinpoint.getPosX() / MM_PER_IN;
            poseIn.y = pinpoint.getPosY() / MM_PER_IN;
            poseIn.heading = wrap(pinpoint.getHeading());

            // velocities (mm/s → in/s, rad/s stays rad/s)
            vxIn = pinpoint.getVelX() / MM_PER_IN;
            vyIn = pinpoint.getVelY() / MM_PER_IN;
            omegaRad = pinpoint.getHeadingVelocity();

        } else {
            // cheaper: only heading
            pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            poseIn.heading = wrap(pinpoint.getHeading());
        }
    }

    /** Current pose (inches, radians). */
    public Pose2d getPose(){ return poseIn; }

    /** Linear velocity (in/s) and angular velocity (rad/s). */
    public double getVX(){ return vxIn; }
    public double getVY(){ return vyIn; }
    public double getOmega(){ return omegaRad; }

    /** Device state helpers for telemetry. */
    public String getStatusString(){
        if (pinpoint == null || pinpoint.getDeviceStatus() == null) return "NOT_CONNECTED";
        return pinpoint.getDeviceStatus().name();
    }
    public double getDeviceHz(){ return (pinpoint != null) ? pinpoint.getFrequency() : 0.0; }
    public int getLoopTimeMicros(){ return (pinpoint != null) ? pinpoint.getLoopTime() : 0; }

    // ===================== EXTERNAL POSE FUSION =====================

    /**
     * Blend an external absolute pose (e.g., AprilTag/Limelight) into Pinpoint smoothly.
     * alpha ∈ [0..1]: 0=no change, 1=snap to external.
     * Writes the blended pose back to the device so its internal fusion continues from the corrected state.
     */
    public void fuseExternalPose(Pose2d ext, double alpha){
        if (pinpoint == null || ext == null) return;
        alpha = clamp(alpha, 0.0, 1.0);

        Pose2d cur = getPose();

        double bx = cur.x + alpha * (ext.x - cur.x);
        double by = cur.y + alpha * (ext.y - cur.y);
        double bh = wrap(cur.heading + alpha * angleDiff(ext.heading, cur.heading));

        setPose(new Pose2d(bx, by, bh));
    }

    // ===================== utils =====================

    private static double wrap(double a){
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }

    /** Smallest signed angle from 'from' to 'to' (radians, wrapped). */
    private static double angleDiff(double to, double from){
        return wrap(to - from);
    }

    private static double clamp(double v, double lo, double hi){
        return Math.max(lo, Math.min(hi, v));
    }
}