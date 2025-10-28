package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.teamcode.programs.utils.Robot.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Turret extends SubsystemBase {
    private Servo servoX, servoY;
    private Limelight3A limelight;

    private final double centerX = 0.5;
    private final double centerY = 0.5;
    private final double upY = 0.75;
    private final double downY = 0.25;

    private final double yawMaxDeg = 150.0;
    private final double pitchMaxDeg = 20.0;
    private final double cameraHeightM = 0.2925;

    private final double CAMERA_ANGLE = 20;
    private final double CAMERA_HEIGHT = 0.2925;
    private final double LATERAL_OFFSET = 0.0;
    private final double BONUS = 0.01;

    private final double LIMELIGHT_GAIN = 0.0005;
    private final double GEAR_RATIO_YAW = 1.5;
    private final double GEAR_RATIO_PITCH = 1.0;

    public static class TagPose {
        public final double x, y, z;
        public TagPose(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    private static final Map<Integer, TagPose> FIELD_TAGS = new HashMap<>();
    static {
        FIELD_TAGS.put(20, new TagPose(2.98, 1.31, 0.749));
        FIELD_TAGS.put(24, new TagPose(1.482, -1.413, 0.749));
    }

    private TagPose lockedTag = null;
    private double limelightOffsetX = 0.0, limelightOffsetY = 0.0;

    public void initialize() {
        Robot robot = Robot.getInstance();
        servoX = robot.servoX;
        servoY = robot.servoY;
        limelight = robot.limelight;
        servoX.setPosition(centerX);
        servoY.setPosition(centerY);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop(int targetID) {
        GoBildaPinpointDriver pinpoint = Robot.getInstancePinpoint();
        if (pinpoint == null || !FIELD_TAGS.containsKey(targetID)) return;

        pinpoint.update();
        double robotX = pinpoint.getPosX(DistanceUnit.METER);
        double robotY = pinpoint.getPosY(DistanceUnit.METER);
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        if (lockedTag == null) lockedTag = FIELD_TAGS.get(targetID);

        aimAt(lockedTag, robotX, robotY, robotHeading);
        applyLimelightCorrection(targetID);
    }

    private void aimAt(TagPose tag, double robotX, double robotY, double robotHeading) {
        double dx = tag.x - robotX;
        double dy = tag.y - robotY;
        double dz = tag.z - cameraHeightM;

        double angleToTag = Math.atan2(dy, dx);
        double relativeYaw = wrapRad(angleToTag - robotHeading);
        relativeYaw = clamp(relativeYaw, -Math.toRadians(yawMaxDeg), Math.toRadians(yawMaxDeg));

        double servoXTarget = centerX + (relativeYaw / Math.toRadians(yawMaxDeg)) / GEAR_RATIO_YAW + limelightOffsetX;

        double horiz = Math.hypot(dx, dy);
        double pitchRad = Math.atan2(dz, horiz);

        double servoYTarget = centerY + (Math.toDegrees(pitchRad) / pitchMaxDeg) / GEAR_RATIO_PITCH * (upY - centerY) + limelightOffsetY;

        servoX.setPosition(clamp(servoXTarget, 0.0, 1.0));
        servoY.setPosition(clamp(servoYTarget, downY, upY));
    }

    private void applyLimelightCorrection(int targetID) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetID) {
                double tx = tag.getTargetXDegreesNoCrosshair();
                double ty = tag.getTargetYDegreesNoCrosshair();

                if (Math.abs(tx) < 0.5 && Math.abs(ty) < 0.5) return;

                double cameraAngleRad = Math.toRadians(CAMERA_ANGLE + ((servoY.getPosition() - centerY) / (upY - centerY)) * pitchMaxDeg);
                double yDistance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty) + cameraAngleRad);

                double xDistance = Math.sqrt(yDistance*yDistance + CAMERA_HEIGHT*CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) + LATERAL_OFFSET;
                double targetAngle = -Math.atan2(xDistance, yDistance + BONUS);
                
                limelightOffsetX = clamp(targetAngle * LIMELIGHT_GAIN / GEAR_RATIO_YAW / Math.toRadians(yawMaxDeg), -0.1, 0.1);
                limelightOffsetY = clamp(ty * 0.0015 / GEAR_RATIO_PITCH, -0.1, 0.1);

                telemetry.addData("tx:", tag.getTargetXDegreesNoCrosshair());
                telemetry.addData("ty:", tag.getTargetYDegreesNoCrosshair());
                telemetry.addData("target angle:", targetAngle);
                telemetry.update();

                break;
            }
        }
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2*Math.PI;
        while (r < -Math.PI) r += 2*Math.PI;
        return r;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
