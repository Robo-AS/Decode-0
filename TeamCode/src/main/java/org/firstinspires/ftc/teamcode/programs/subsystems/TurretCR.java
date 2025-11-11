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

    public TurretCR() {
        servoX = robot.servoX;
        limelight = robot.limelight;

        // if current angle is in range of +- 4 degrees of target it doesnt correct
        // should remove jitter
        pid.setTolerance(TOLERANCE);
    }

    public void initialize(){
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
    }


    //the analog input returns voltage between 0 and 3.3 V
    //convert that % of voltage to % of rotation, then to degrees
    private double getRawAngle() {
        double pct = robot.axonEncoder.getVoltage() / robot.axonEncoder.getMaxVoltage();
        return pct * 360.0 / GEAR_RATIO;
    }

    //normalize angle in range [-180, 180] and clamp it too
    private double getAngle() {
        double angle = getRawAngle() - encoderZero;

        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;

        if (angle > MAX_ANGLE) angle = MAX_ANGLE;
        if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;

        return angle;
    }

    //stores the current encoder position as the new "zero", it shifts reference
    public void resetEncoderPosition() {
        encoderZero = getRawAngle();
    }

    private void updateHeadings() {
        // use limelight as much as possible to get the target angle
        // keeping pinpoint as a fail-safe
        LLResult ll = limelight.getLatestResult();
        boolean useLL = ll != null && ll.isValid() && ll.getBotpose_MT2() != null && FIELD_TAGS_LL.containsKey(targetID);
        boolean usePinpoint = robot.pinpoint != null && FIELD_TAGS_LL.containsKey(targetID) && !useLL;

        double robotX, robotY, headingRad;
        TagPose tag;

        if (useLL)
        {
            targetAngle = ll.getTx();

            //get robot field-relative coordinates from limelight as much as possible
            //and sync odometry with those, bc they are more accurate
            //so if you do fallback to pinpoint it has less error (hopefully)
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

            //read robot position and heading from pinpoint
            robotX = robot.pinpoint.getPosX(DistanceUnit.METER);
            robotY = robot.pinpoint.getPosY(DistanceUnit.METER);
            headingRad = Math.toRadians(robot.pinpoint.getHeading(AngleUnit.DEGREES));

            //angle to target tag in world space
            tag = FIELD_TAGS_LL.get(targetID);
            double dx = tag.x - robotX;
            double dy = tag.y - robotY;

            //get the target angle and normalize it
            double fieldAngle = Math.atan2(dy, dx);
            double error = fieldAngle - headingRad; // angle from goal - current heading
            double errorDeg = Math.toDegrees(error);

            while (errorDeg > 180) errorDeg -= 360;
            while (errorDeg < -180) errorDeg += 360;

            targetAngle = errorDeg;
        }
        else targetAngle = 0; //if neither is available for some reason reset the turret
    }


    public void loop() {
        updateHeadings(); //get targetAngle

        double current = getAngle();

        //normalize again bc i only do that for pinpoint
        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        //clamp it again to [-180, 180] for extra safety
        targetAngle = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, targetAngle));

        //new pid and feedforward
        pid.setPIDF(kP, kI, kD, 0);
        ff = new SimpleMotorFeedforward(kS, kV);

        double pidOut = pid.calculate(current, targetAngle); //get the target power
        double ffOut = ff.calculate(targetAngle - current); //feedforward based on angular error rate
        double power = pidOut + ffOut; // combo

        if(pid.atSetPoint())
            //using the setTolerance method only works if you are going to use atSetPoint
            //basically the setPoint isnt just one degree anymore, its an interval:
            //[targetAngle-TOLERANCE, targetAngle+TOLERANCE]
            //if you reached that interval stop aiming
            servoX.setPower(0);
        else
            servoX.setPower(power);
    }

}
