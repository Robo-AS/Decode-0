package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Turret extends SubsystemBase {
    private Servo servoX, servoY;
    private Limelight limelight;

    private final double centerX = 0.5;
    private final double centerY = 0.5;
    private final double upY = 0.65;
    private final double downY = 0.25;
    private final double usableHalfX = 0.5;
    private final double usableHalfY = upY - centerY;

    private final double yawMaxDeg = 150.0;
    private final double pitchMaxDeg = 20.0;
    private final double cameraHeightM = 0.2925;

    private final double smoothing = 0.15;
    private double servoXTarget = centerX;
    private double servoYTarget = centerY;

    public static class TagPose {
        public final double x, y, z;
        public TagPose(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }
    }

    private TagPose lockedTag = null;
    private int sampleCount = 0;
    private double sumX, sumY, sumZ;

    public void initialize() {
        Robot robot = Robot.getInstance();
        this.servoX = robot.servoX;
        this.servoY = robot.servoY;
        this.limelight = robot.getInstanceLimelight();

        servoX.setPosition(centerX);
        servoY.setPosition(centerY);
    }

    public void loop(int targetID) {
        limelight.useAprilTagPipeline();
        GoBildaPinpointDriver pinpoint = Robot.pinpoint;
        if (pinpoint == null) return;

        pinpoint.update();
        double robotX = pinpoint.getPosX(DistanceUnit.METER);
        double robotY = pinpoint.getPosY(DistanceUnit.METER);
        double robotHeadingRad = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        if (lockedTag == null && limelight.tagId == targetID) {
            double tx = limelight.getTagXMeters(0.7075);
            double ty = limelight.getTagYMeters(0.7075);
            double tz = limelight.getTagZMeters(0.7075);

            double cosH = Math.cos(robotHeadingRad);
            double sinH = Math.sin(robotHeadingRad);
            double tagX = robotX + tx * cosH - ty * sinH;
            double tagY = robotY + tx * sinH + ty * cosH;
            double tagZ = tz;

            sumX += tagX;
            sumY += tagY;
            sumZ += tagZ;
            sampleCount++;

            if (sampleCount >= 25) {
                lockedTag = new TagPose(sumX / sampleCount, sumY / sampleCount, sumZ / sampleCount);
            }
        }

        if (lockedTag != null) {
            aimAt(lockedTag);
        }

        double curX = servoX.getPosition();
        double curY = servoY.getPosition();
        curX += smoothing * (servoXTarget - curX);
        curY += smoothing * (servoYTarget - curY);
        servoX.setPosition(curX);
        servoY.setPosition(curY);
    }

    private void aimAt(TagPose tag) {
        GoBildaPinpointDriver pinpoint = Robot.pinpoint;
        if (pinpoint == null) return;

        pinpoint.update();
        double robotX = pinpoint.getPosX(DistanceUnit.METER);
        double robotY = pinpoint.getPosY(DistanceUnit.METER);
        double robotHeadingRad = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        double dx = tag.x - robotX;
        double dy = tag.y - robotY;
        double dz = tag.z - cameraHeightM;

        double angleToTag = Math.atan2(dy, dx);
        double relativeRad = wrapRad(angleToTag - robotHeadingRad);

        relativeRad = clamp(relativeRad, -Math.toRadians(yawMaxDeg), Math.toRadians(yawMaxDeg));

        double servoXPos = centerX + (relativeRad / Math.toRadians(yawMaxDeg)) * usableHalfX;
        servoXTarget = clamp(servoXPos, 0.0, 1.0);

        double horiz = Math.hypot(dx, dy);
        double pitchRad = Math.atan2(dz, horiz);

        double servoYPos = centerY + (Math.toDegrees(pitchRad) / pitchMaxDeg) * usableHalfY;
        servoYTarget = clamp(servoYPos, downY, upY);
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
