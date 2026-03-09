package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
public class TurretCR extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final PIDController turretPID;

    public enum TurretState {
        GOAL_LOCK,
        LIMELIGHT_LOCK,
        MANUAL
    }

    public TurretState currentState = TurretState.GOAL_LOCK;

    public static double kP = 0.0125, kI = 0, kD = 0.0005, kS = 0.075;
    public static double targetTurretPosition = 0;
    public static double currentTurretPosition = 0;
    public static double MAX_ANGLE = 360.0, MIN_ANGLE = -90.0;

    private final double TICKS_PER_REV = 8192.0;
    private final double POSITION_TOLERANCE = 0.5;

    private double lastPower = 0;

    public static double staticLastAutoX = 0, staticLastAutoY = 0;
    private double angleToGoalField;

    private boolean isNormalized = false;

    public TurretCR() {
        turretPID = new PIDController(kP, kI, kD);
    }

    public void initialize() {
        robot.turretServo.setPower(0);
        turretPID.reset();
        targetTurretPosition = 0;
        currentState = TurretState.GOAL_LOCK;
    }


    @Override
    public void periodic() {
        currentTurretPosition = (robot.intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;
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

        targetTurretPosition = Math.toDegrees(diff) + driverOffset;
    }

    public void loopAuto(boolean isBottomRed, Pose pedro, double gX, double gY) {
        double robotHeading = pedro.getHeading() - Math.toRadians(90);
        double dX = gX - pedro.getX();
        double dY = gY - pedro.getY();

        angleToGoalField = (dY != 0) ? Math.atan(dX / dY) : 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetTurretPosition = Math.toDegrees(diff);
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
                targetTurretPosition = currentTurretPosition + ll.getTx();
            }
        }
    }

    private void applyToHardware() {
        while (targetTurretPosition - currentTurretPosition > 180) {
            isNormalized = true;
            kP = 0.125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
            targetTurretPosition -= 360;
        }

        while (targetTurretPosition - currentTurretPosition < -180) {
            isNormalized = true;
            kP = 0.125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
            targetTurretPosition += 360;
        }

        if (targetTurretPosition < MIN_ANGLE) {
            isNormalized = true;
            kP = 0.125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
            targetTurretPosition += 360;
        }
        if (targetTurretPosition > MAX_ANGLE) {
            isNormalized = true;
            kP = 0.125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
            targetTurretPosition -= 360;
        }

        targetTurretPosition = Range.clip(targetTurretPosition, MIN_ANGLE, MAX_ANGLE);

        if(isNormalized && Math.abs(currentTurretPosition - targetTurretPosition) < 5.0) {
            isNormalized = false;
        }

        if(!isNormalized)
            updatePIDCoefficients();

        turretPID.setPID(kP, kI, kD);

        double error = targetTurretPosition - currentTurretPosition;
        double newPower = 0;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            newPower = turretPID.calculate(currentTurretPosition, targetTurretPosition) + (Math.signum(error) * kS);
        }

        if (Math.abs(newPower - lastPower) > 0.005) {
            robot.turretServo.setPower(newPower);
            lastPower = newPower;
        }

        robot.turretServo.setPower(newPower);
    }

    public void loop(double goalX, double goalY, TurretState state, int targetID, boolean isRed, double robotX, double robotY, double driverOffset) {
        this.currentState = state;
        if (currentState == TurretState.GOAL_LOCK) {
            updateGoalLock(goalX, goalY, robotX, robotY, driverOffset);
        }
        else if (currentState == TurretState.LIMELIGHT_LOCK) {
            updateLimelight(targetID);
        }

        currentTurretPosition = (robot.intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        applyToHardware();
    }


    public void loopTest(){
        currentTurretPosition = (robot.intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        turretPID.setPID(kP, kI, kD);
        double error = targetTurretPosition - currentTurretPosition;
        double power = turretPID.calculate(currentTurretPosition, targetTurretPosition) + Math.signum(error) * kS;
        robot.turretServo.setPower(power);
    }

    public double getCurrentAngle() {
        return currentTurretPosition;
    }

    public void setTargetRotation(double target) {
        targetTurretPosition = target;
    }

    public double getTargetAngle() {
        return targetTurretPosition;
    }

    public void updatePIDCoefficients()
    {
        if(currentTurretPosition <= targetTurretPosition) //clockwise
        {
            if(targetTurretPosition >= -90 && targetTurretPosition <= 80) {
                kP = 0.018;
                kI = 0.01;
                kD = 0;
                kS = 0.069;
            }
            else if(targetTurretPosition > 80 && targetTurretPosition <= 280){
                kP = 0.022;
                kI = 0.011;
                kD = 0;
                kS = 0.06;
            }
            else if(targetTurretPosition > 280 && targetTurretPosition <= 310){
                kP = 0.04;
                kI = 0.028;
                kD = 0;
                kS = 0.06;
            }
            else if(targetTurretPosition > 310 && targetTurretPosition <= 360){
                kP = 0.044;
                kI = 0.026;
                kD = 0;
                kS = 0.06;
            }
        }
        else{
            kP = 0.0125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
        }
    }
}