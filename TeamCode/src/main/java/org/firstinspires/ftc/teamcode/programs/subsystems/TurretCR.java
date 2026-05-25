package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.programs.test.TurretPIDTuning;
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
    public static double POSITION_TOLERANCE = 0.0;

    private double lastPower = 0;

    public static double staticLastAutoX = 7.874015748, staticLastAutoY = 8.661417323;
    public double resetOffset = 0;
    private double angleToGoalField;

    private boolean isNormalized = false;

    public static double ANGLE_DIFFERENCE = 5;

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

        double distance = 3.2677165354;

        double turretOffsetX = -distance * Math.cos(robotHeading);
        double turretOffsetY = distance * Math.sin(robotHeading);

        double turretX = robotX + turretOffsetX;
        double turretY = robotY + turretOffsetY;

        double dX = (goalX - staticLastAutoX) - turretX;
        double dY = (goalY - staticLastAutoY) - turretY;

        angleToGoalField = (dX != 0) ? Math.atan2(dY, dX) : 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        double interpolationOffset = 0.0;
        double distanceToTarget = Math.hypot(dX, dY);

        if (distanceToTarget > 100.0) {
            double headingDegrees = Math.toDegrees(robotHeading);

            double oldMath = 8.0 * Math.cos(robotHeading);

            double fixedMath = -8.0 * Math.abs(Math.cos(robotHeading - (Math.PI / 4.0)));

            if (headingDegrees >= -135.0 && headingDegrees <= 45.0) {
                double center = -45.0;
                double halfWidth = 90.0;

                double distanceFromCenter = Math.abs(headingDegrees - center);
                double blendFactor = Math.max(0.0, 1.0 - (distanceFromCenter / halfWidth));

                interpolationOffset = (blendFactor * fixedMath) + ((1.0 - blendFactor) * oldMath);
            } else {
                interpolationOffset = oldMath;
            }
        }

        targetTurretPosition = Math.toDegrees(diff) + driverOffset + interpolationOffset - resetOffset;
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

    public void loopAutomated(double target){
        targetTurretPosition = target;
        applyToHardware();
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
        if (targetTurretPosition < MIN_ANGLE) {
            isNormalized = true;
            kP = 0.0125;
            kI = 0;
            kD = 0.0005;
            kS = 0.075;
            targetTurretPosition += 360;
        }
        if (targetTurretPosition > MAX_ANGLE) {
            isNormalized = true;
            kP = 0.0125;
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

        if (Math.abs(error) >= POSITION_TOLERANCE) {
            newPower = turretPID.calculate(currentTurretPosition, targetTurretPosition) + (Math.signum(error) * kS);
        }

        if (Math.abs(newPower - lastPower) > 0.005) {
            robot.turretServo.setPower(newPower);
            robot.sec_turretServo.setPower(newPower);
            lastPower = newPower;
        }
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

    public void applyBlueResetClose_BLUE(){
        robot.pinpoint.resetPosAndIMU();
        Robot.getInstance().driverOffset = 0;
        TurretCR.staticLastAutoX = 82.29;
        TurretCR.staticLastAutoY = 15.25;
    }

    public void applyBlueResetFar_BLUE(){
        robot.pinpoint.resetPosAndIMU();
        Robot.getInstance().driverOffset = 0;
        TurretCR.staticLastAutoX = 7.874015748;
        TurretCR.staticLastAutoY = 135.338582677;
    }

    public void applyBlueResetClose_RED(){
        robot.pinpoint.resetPosAndIMU();
        Robot.getInstance().driverOffset = 0;
        TurretCR.staticLastAutoX = 82.29;
        TurretCR.staticLastAutoY = 128.75;
    }

    public void applyBlueResetFar_RED(){
        robot.pinpoint.resetPosAndIMU();
        Robot.getInstance().driverOffset = 0;
        TurretCR.staticLastAutoX = 7.874015748;
        TurretCR.staticLastAutoY =  8.661417323;
    }

    public double getCurrentAngle() { return currentTurretPosition; }
    public void setTargetRotation(double target) { targetTurretPosition = target; }
    public double getTargetAngle() { return targetTurretPosition; }

    public void updatePIDCoefficients() {
        if(currentTurretPosition <= targetTurretPosition){//clockwise
            if(Math.abs(currentTurretPosition - targetTurretPosition) <= ANGLE_DIFFERENCE){ //small difference
                if(targetTurretPosition >= -90 && targetTurretPosition <= -50) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition > -50 && targetTurretPosition <= -25) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition > -25 && targetTurretPosition <= 80) {
                    kP = 0.0155; kI = 0.05; kD = 0.0006; kS = 0.06;
                }
                else if(targetTurretPosition > 80 && targetTurretPosition <= 280){
                    kP = 0.022; kI = 0.011; kD = 0; kS = 0.06;
                }
                else if(targetTurretPosition > 280 && targetTurretPosition <= 300){
                    kP = 0.034; kI = 0.035; kD = 0; kS = 0.06;
                }
                else if(targetTurretPosition > 300 && targetTurretPosition <= 320){
                    kP = 0.035; kI = 0.02; kD = 0; kS = 0.06;
                }
                else if(targetTurretPosition > 320 && targetTurretPosition <= 325){
                    kP = 0.034; kI = 0.033; kD = 0; kS = 0.06;
                }
                else if(targetTurretPosition > 325 && targetTurretPosition <= 330){
                    kP = 0.035; kI = 0.029; kD = 0; kS = 0.05;
                }
                else if(targetTurretPosition > 330 && targetTurretPosition <= 345){
                    kP = 0.049; kI = 0.03; kD = 0.00009; kS = 0.05;
                }
                else if(targetTurretPosition > 345 && targetTurretPosition <= 360){
                    kP = 0.06; kI = 0.04; kD = 0.00009; kS = 0.06;
                }
            }
            else{
                kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
            }
        }
        else{
            if(Math.abs(currentTurretPosition - targetTurretPosition) <= ANGLE_DIFFERENCE) {
                if(targetTurretPosition >= -90 && targetTurretPosition < -50) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition >= -50 && targetTurretPosition < -45) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition >= -45 && targetTurretPosition < -40) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition >= -40 && targetTurretPosition < -25) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition >= -25 && targetTurretPosition < -20) {
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition >= -20 && targetTurretPosition <= 80) {
                    kP = 0.0155; kI = 0.05; kD = 0.0006; kS = 0.06;
                }
                else if (targetTurretPosition > 80 && targetTurretPosition <= 280){
                    kP = 0.022; kI = 0.011; kD = 0; kS = 0.053;
                }
                else if(targetTurretPosition > 280 && targetTurretPosition <= 300){
                    kP = 0.02; kI = 0.015; kD = 0; kS = 0.065;
                }
                else if(targetTurretPosition > 300 && targetTurretPosition <= 320){
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition > 320 && targetTurretPosition <= 325){
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else if(targetTurretPosition > 325 && targetTurretPosition <= 330){
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
                else{
                    kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
                }
            }
            else{
                kP = 0.015; kI = 0.005; kD = 0.001; kS = 0.06;
            }
        }
    }
}

//    public void updatePIDCoefficients() {
//        kP = 0.015;
//        kI = 0.005;
//        kD = 0.001;
//        kS = 0.06;
//    }
