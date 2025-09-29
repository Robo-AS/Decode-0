package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import java.util.HashMap;
import java.util.Map;

public class Turret extends SubsystemBase {
    private Servo servoX, servoY;

    private final double centerX = 0.5;
    private final double centerY = 0.5;
    private final double upY = 0.75;
    private final double downY = 0.25;

    private final double yawMaxDeg = 150.0;
    private final double pitchMaxDeg = 20.0;
    private final double cameraHeightM = 0.2925;

    public static class TagPose {
        public final double x, y, z;
        public TagPose(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }
    }

    private static final Map<Integer, TagPose> FIELD_TAGS = new HashMap<>();
    static {
        FIELD_TAGS.put(20, new TagPose(-1.482, -1.413, 0.749));
        FIELD_TAGS.put(24, new TagPose(-1.482,  1.413, 0.749));
    }

    private TagPose lockedTag = null;

    public void initialize() {
        Robot robot = Robot.getInstance();

        servoX = robot.servoX;
        servoY = robot.servoY;

        servoX.setPosition(centerX);
        servoY.setPosition(centerY);
    }

    public void loop(int targetID) {
        GoBildaPinpointDriver pinpoint = Robot.getInstancePinpoint();
        if (pinpoint == null || !FIELD_TAGS.containsKey(targetID)) return;

        pinpoint.update();
        double robotX = pinpoint.getPosX(DistanceUnit.METER);
        double robotY = pinpoint.getPosY(DistanceUnit.METER);
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        if (lockedTag == null) {
            lockedTag = FIELD_TAGS.get(targetID);
        }

        aimAt(lockedTag, robotX, robotY, robotHeading);
    }

    private void aimAt(TagPose tag, double robotX, double robotY, double robotHeading) {
        double dx = tag.x - robotX;
        double dy = tag.y - robotY;
        double dz = tag.z - cameraHeightM;

        double angleToTag = Math.atan2(dy, dx);
        double relativeAngle = wrapRad(angleToTag - robotHeading);
        relativeAngle = clamp(relativeAngle, -Math.toRadians(yawMaxDeg), Math.toRadians(yawMaxDeg));
        double servoXPos = centerX + (relativeAngle / Math.toRadians(yawMaxDeg)) * 0.5;
        servoXPos = clamp(servoXPos, 0.0, 1.0);
        servoX.setPosition(servoXPos);

        double horiz = Math.hypot(dx, dy);
        double pitchRad = Math.atan2(dz, horiz);
        double servoYPos = centerY + (Math.toDegrees(pitchRad) / pitchMaxDeg) * (upY - centerY);
        servoYPos = clamp(servoYPos, downY, upY);
        servoY.setPosition(servoYPos);
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
