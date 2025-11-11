package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import java.util.HashMap;
import java.util.Map;

public class TurretCR extends SubsystemBase {

    public enum Mode {
        HOMING,
        AIMING,
        RAW,
        FIXED
    }

    private final Robot robot = Robot.getInstance();
    private CRServo servoX;
    private Limelight3A limelight;

    public static Mode mode = Mode.RAW;
    public static double targetPower = 0.0;
    public static double targetAngle = 0.0;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double kV = 0.0;

    public static double GEAR_RATIO = 1.5;
    public static double MAX_ANGLE = 180.0;
    public static double TOLERANCE = 4.0;

    private PIDFController pid = new PIDFController(kP, kI, kD, 0);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV);

    private double encoderZero = 0.0;
    private double robotHeading = 0.0;
    private double goalHeading = 0.0;
    private int targetID = 20;

    public static class TagPose { public final double x,y,z; TagPose(double x,double y,double z){this.x=x;this.y=y;this.z=z;} }

    private static final Map<Integer, TagPose> FIELD_TAGS_LL = new HashMap<>();
    static {
        FIELD_TAGS_LL.put(20, new TagPose(-1.482, -1.413, 0.749));
        FIELD_TAGS_LL.put(24, new TagPose(-1.482, 1.413, 0.749));
    }

    private static final Map<Integer, TagPose> FIELD_TAGS_PINPOINT = new HashMap<>();
    static {
        FIELD_TAGS_PINPOINT.put(20, new TagPose(2.98, 1.31, 0.749));
        FIELD_TAGS_PINPOINT.put(24, new TagPose(2.98, -1.31, 0.749));
    }

    public TurretCR() {
        servoX = robot.servoX;
        limelight = robot.limelight;
        pid.setTolerance(TOLERANCE);
    }

    public void initialize(){
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
    }


    private double getRawAngle() {
        double pct = robot.axonEncoder.getVoltage() / robot.axonEncoder.getMaxVoltage();
        return pct * 360.0 / GEAR_RATIO;
    }

    private double getAngle() {
        double angle = getRawAngle() - encoderZero;

        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;

        if (angle > MAX_ANGLE) angle = MAX_ANGLE;
        if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;

        return angle;
    }

    public void resetEncoderPosition() {
        encoderZero = getRawAngle();
    }

    private void updateHeadings() {
        LLResult ll = limelight.getLatestResult();
        boolean useLL = ll != null && ll.isValid() && ll.getBotpose_MT2() != null && FIELD_TAGS_LL.containsKey(targetID);
        boolean usePinpoint = robot.pinpoint != null && FIELD_TAGS_LL.containsKey(targetID) && !useLL;

        double robotX, robotY, headingRad;
        TagPose tag;

        if (useLL)
        {
            targetAngle = ll.getTx();
            Pose2D poseFromLL = new Pose2D(
                    DistanceUnit.METER,
                    ll.getBotpose_MT2().getPosition().x,
                    ll.getBotpose_MT2().getPosition().y,
                    AngleUnit.RADIANS,
                    ll.getBotpose_MT2().getOrientation().getYaw()
            );
            robot.pinpoint.setPosition(poseFromLL);
        }
        else if (usePinpoint)
        {
            robot.pinpoint.update();

            robotX = robot.pinpoint.getPosX(DistanceUnit.METER);
            robotY = robot.pinpoint.getPosY(DistanceUnit.METER);
            headingRad = Math.toRadians(robot.pinpoint.getHeading(AngleUnit.DEGREES));

            tag = FIELD_TAGS_LL.get(targetID);
            double dx = tag.x - robotX;
            double dy = tag.y - robotY;

            double fieldAngle = Math.atan2(dy, dx);
            double error = fieldAngle - headingRad;
            double errorDeg = Math.toDegrees(error);

            while (errorDeg > 180) errorDeg -= 360;
            while (errorDeg < -180) errorDeg += 360;

            targetAngle = errorDeg;
        }
        else targetAngle = 0;
    }


    public void loop() {
        updateHeadings();

        double current = getAngle();

        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        targetAngle = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, targetAngle));

        pid.setPIDF(kP, kI, kD, 0);
        ff = new SimpleMotorFeedforward(kS, kV);

        double pidOut = pid.calculate(current, targetAngle);
        double ffOut = ff.calculate(targetAngle - current);
        double power = pidOut + ffOut;

        if(pid.atSetPoint())
            servoX.setPower(0);
        else
            servoX.setPower(power);
    }

}
