package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class TurretCR extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final CRServo servo;
    private final DcMotorEx encoderMotor;
    private final PIDController pid;

    public enum TurretState { GOAL_LOCK, LIMELIGHT_LOCK, MANUAL }
    public TurretState currentState = TurretState.GOAL_LOCK;

    public static double kP = 0.0125, kI = 0, kD = 0.0005, kS = 0.075;
    public static double targetAngle = 0;
    public static double MAX_ANGLE = 360.0, MIN_ANGLE = -90.0;

    private final double TICKS_PER_REV = 8192.0;
    private final double POSITION_TOLERANCE = 0.5;

    private double lastPower = 0;
    private double currentAngleCache = 0;

    public static double staticLastAutoX = 0, staticLastAutoY = 0;
    private double angleToGoalField;

    public TurretCR(CRServo servo, DcMotorEx encoderMotor) {
        this.servo = servo;
        this.encoderMotor = encoderMotor;
        this.pid = new PIDController(kP, kI, kD);
    }

    public void initialize() {
        servo.setPower(0);
        pid.reset();
        targetAngle = 0;
    }

    @Override
    public void periodic() {
        currentAngleCache = (encoderMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        applyToHardware();
    }

    private void updateGoalLock(double goalX, double goalY, double robotX, double robotY, double driverOffset) {
        double robotHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double dX = (goalX - staticLastAutoX) - robotX;
        double dY = (goalY - staticLastAutoY) - robotY;

        angleToGoalField = (dX != 0) ? Math.atan2(dY, dX) : 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetAngle = Math.toDegrees(diff) + driverOffset;
    }

    public void loopAuto(boolean isBottomRed, Pose pedro, double gX, double gY) {
        double robotHeading = pedro.getHeading() - Math.toRadians(90);
        double dX = gX - pedro.getX();
        double dY = gY - pedro.getY();

        angleToGoalField = (dY != 0) ? Math.atan(dX / dY) : 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetAngle = Math.toDegrees(diff);
    }

    private void updateLimelight(int targetID) {
        LLResult ll = robot.limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            boolean found = false;
            if (ll.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult ft : ll.getFiducialResults()) {
                    if (ft.getFiducialId() == targetID) {
                        found = true;
                        break;
                    }
                }
            }
            if (found) {
                targetAngle = currentAngleCache + ll.getTx();
            }
        }
    }

    private void applyToHardware() {
        double current = currentAngleCache;
        double tempTarget = targetAngle;

        while (tempTarget - current > 180) tempTarget -= 360;
        while (tempTarget - current < -180) tempTarget += 360;

        if (tempTarget < MIN_ANGLE) tempTarget += 360;
        if (tempTarget > MAX_ANGLE) tempTarget -= 360;

        targetAngle = Range.clip(tempTarget, MIN_ANGLE, MAX_ANGLE);

        pid.setPID(kP, kI, kD);
        double error = targetAngle - current;
        double newPower = 0;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            newPower = pid.calculate(current, targetAngle) + (Math.signum(error) * kS);
        }

        if (Math.abs(newPower - lastPower) > 0.005) {
            servo.setPower(newPower);
            lastPower = newPower;
        }
    }

    public void loop(double goalX, double goalY, TurretState mode, int targetID, boolean isRed, double robotX, double robotY, double driverOffset) {
        this.currentState = mode;
        if (currentState == TurretState.GOAL_LOCK) {
            updateGoalLock(goalX, goalY, robotX, robotY, driverOffset);
        } else if (currentState == TurretState.LIMELIGHT_LOCK) {
            updateLimelight(targetID);
        }
    }

    public double getCurrentAngle() {
        return currentAngleCache;
    }

    public void setTargetRotation(double target) {
        targetAngle = target;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void update() {
        applyToHardware();
    }
}