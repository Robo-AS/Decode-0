package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.teamcode.programs.utils.Robot.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import java.util.HashMap;
import java.util.Map;
import java.util.LinkedList;
import java.util.Queue;

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
    private final double GEAR_RATIO_YAW = 1.5;

    private static final int SMOOTH_FRAMES = 100;
    private final Queue<Double> llXQueue = new LinkedList<>();
    private final Queue<Double> llYQueue = new LinkedList<>();
    private final Queue<Double> llHeadingQueue = new LinkedList<>();

    private static final double MAX_LL_DELTA = 0.5;
    private static final double Kp = 0.2;
    private static final double MAX_SERVO_DELTA = 0.02;
    private static final double MIN_ERROR = 0.002;

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
        if (lockedTag == null) lockedTag = FIELD_TAGS.get(targetID);

        double robotX = pinpoint.getPosX(DistanceUnit.METER);
        double robotY = pinpoint.getPosY(DistanceUnit.METER);
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double[] pinpointTargets = computeServoTargets(lockedTag, robotX, robotY, robotHeading);
        double[] correctedTargets = blendLimelightCorrection(targetID, pinpointTargets, pinpoint);

        double servoXCurrent = servoX.getPosition();
        double servoYCurrent = servoY.getPosition();

        double deltaX = clamp(correctedTargets[0] - servoXCurrent, -MAX_SERVO_DELTA, MAX_SERVO_DELTA);
        double deltaY = clamp(correctedTargets[1] - servoYCurrent, -MAX_SERVO_DELTA, MAX_SERVO_DELTA);

        if(Math.abs(deltaX) > MIN_ERROR) servoX.setPosition(clamp(servoXCurrent + deltaX, 0.0, 1.0));
        if(Math.abs(deltaY) > MIN_ERROR) servoY.setPosition(clamp(servoYCurrent + deltaY, downY, upY));
    }

    private double[] computeServoTargets(TagPose tag, double robotX, double robotY, double robotHeading) {
        double dx = tag.x - robotX;
        double dy = tag.y - robotY;
        double dz = tag.z - cameraHeightM;

        double horiz = Math.hypot(dx, dy);
        double distance = Math.max(horiz, 0.05);

        double angleToTag = Math.atan2(dy, dx);
        double relativeYaw = wrapRad(angleToTag - robotHeading);
        double servoXTarget = centerX + (relativeYaw / Math.toRadians(yawMaxDeg)) / GEAR_RATIO_YAW;

        double pitchRad = Math.atan2(dz, horiz);
        double basePitch = Math.toDegrees(pitchRad) / pitchMaxDeg;

        double distanceComp = clamp((1.2 - distance) * 0.25, 0.0, 0.15);
        basePitch -= distanceComp;

        double servoYTarget = centerY + basePitch * (upY - downY);
        servoYTarget = clamp(servoYTarget, downY, upY);

        telemetry.addData("Distance (m)", distance);
        telemetry.addData("BasePitch", basePitch);
        telemetry.addData("DistanceComp", distanceComp);
        telemetry.addData("servoYTarget", servoYTarget);

        return new double[]{servoXTarget, servoYTarget};
    }



    private double[] blendLimelightCorrection(int targetID, double[] baseTargets, GoBildaPinpointDriver pinpoint) {
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.RADIANS));
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return baseTargets;

        Pose3D botPose = result.getBotpose();
        if (botPose == null) return baseTargets;

        double llX = botPose.getPosition().x;
        double llY = botPose.getPosition().y;
        double llHeading = botPose.getOrientation().getYaw();

        if (!llXQueue.isEmpty() && Math.abs(llX - llXQueue.peek()) > MAX_LL_DELTA) return baseTargets;
        if (!llYQueue.isEmpty() && Math.abs(llY - llYQueue.peek()) > MAX_LL_DELTA) return baseTargets;

        llXQueue.add(llX); if (llXQueue.size() > SMOOTH_FRAMES) llXQueue.poll();
        llYQueue.add(llY); if (llYQueue.size() > SMOOTH_FRAMES) llYQueue.poll();
        llHeadingQueue.add(llHeading); if (llHeadingQueue.size() > SMOOTH_FRAMES) llHeadingQueue.poll();

        double avgX = llXQueue.stream().mapToDouble(d->d).average().orElse(llX);
        double avgY = llYQueue.stream().mapToDouble(d->d).average().orElse(llY);
        double avgHeading = llHeadingQueue.stream().mapToDouble(d->d).average().orElse(llHeading);

        TagPose tag = FIELD_TAGS.get(targetID);
        if (tag == null) return baseTargets;

        double[] llTargets = computeServoTargets(tag, avgX, avgY, avgHeading);
        double servoXError = llTargets[0] - baseTargets[0];
        double servoYError = llTargets[1] - baseTargets[1];

        double confidence = 1.0 / (1.0 + computeStd(llXQueue) + computeStd(llYQueue));

        double correctedX = baseTargets[0] + Kp * confidence * servoXError;
        double correctedY = baseTargets[1] + Kp * confidence * servoYError;

        telemetry.addData("LL Correction", confidence);
        telemetry.addData("ServoX Error", servoXError);
        telemetry.addData("ServoY Error", servoYError);
        telemetry.update();

        return new double[]{correctedX, correctedY};
    }

    private static double computeStd(Queue<Double> q) {
        double mean = q.stream().mapToDouble(d->d).average().orElse(0.0);
        return Math.sqrt(q.stream().mapToDouble(d->(d-mean)*(d-mean)).average().orElse(0.0));
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
