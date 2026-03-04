package org.firstinspires.ftc.teamcode.programs.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import java.util.TreeMap;

public class TurretCR extends SubsystemBase {
    private final Robot robot = Robot.getInstance();


    public enum TurretState {
        GOAL_LOCK,
        LIMELIGHT_LOCK,
        MANUAL
    }


    public TurretState currentState = TurretState.GOAL_LOCK;
    public static double targetAngle = 0;
    public static double kP = 0.0125, kI = 0, kD = 0.0005, kS = 0.075;
    public static double MAX_ANGLE = 360.0, MIN_ANGLE = -90.0;
    public double distance = 0;
    public static double staticLastAutoX = 0, staticLastAutoY = 0; // pozitia la care termina autonomi => translatare in origine
    double angleToGoalField;



    public void initialize() {
        robot.axon.initialize(0);
    }


    @Override
    public void periodic() {
        applyToHardware();
    }


    private void updateGoalLock(double goalX, double goalY, double robotX, double robotY, double driverOffset) {
        double robotHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
        goalX -= staticLastAutoX;
        goalY -= staticLastAutoY; // translatare robot la origine

        if ((goalX - robotX) != 0) //in case the robot is scored targeting still works
            angleToGoalField = Math.atan((goalY - robotY) / (goalX - robotX));
        else
            angleToGoalField = 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetAngle = Math.toDegrees(diff) + driverOffset;

    }


    public void loopAuto(boolean isBottomRed, Pose pedro, double gX, double gY) {
        double robotHeading = pedro.getHeading() - Math.toRadians(90);
        double robotX = pedro.getX();
        double robotY = pedro.getY();
        double goalX = gX;
        double goalY = gY;
        if ((goalY - robotY) != 0)
            angleToGoalField = Math.atan((goalX - robotX) / (goalY - robotY));
        else angleToGoalField = 0;


        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        targetAngle = Math.toDegrees(diff);
        applyToHardware();
    }
    private void updateLimelight(int targetID) {
        LLResult ll = robot.limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            boolean found = false;
            if (ll.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult ft : ll.getFiducialResults()) {
                    if (ft.getFiducialId() == targetID) { found = true; break; }
                }
            }
            if (found) {
                targetAngle = robot.axon.getCurrentAngle() + ll.getTx();
            }
        }
    }


    public void loop(double goalX, double goalY, TurretState mode, int targetID, boolean isRed, double robotX, double robotY, double driverOffset) {
        this.currentState = mode;
        if (currentState == TurretState.GOAL_LOCK)
            updateGoalLock(goalX, goalY, robotX, robotY, driverOffset);
        else if (currentState == TurretState.LIMELIGHT_LOCK)
            updateLimelight(targetID);
    }
    public void loopAuto(double target){
        targetAngle = target;
        applyToHardware();
    }


    private void applyToHardware() {
        double current = robot.axon.getCurrentAngle();
        double target = targetAngle;
        while (target - current > 180) target -= 360;
        while (target - current < -180) target += 360;
        if (target < MIN_ANGLE) target += 360;
        if (target > MAX_ANGLE) target -= 360;
        targetAngle = Range.clip(target, MIN_ANGLE, MAX_ANGLE);
        robot.axon.updatePIDCoeffs(kP, kI, kD, kS);
        robot.axon.setTargetRotation(targetAngle);
        robot.axon.update();
    }


    public double getAngleToGoalField() { return angleToGoalField; }
    public double getTargetAngle() { return targetAngle; }
//    public double getDistance(){ return distance; }
}