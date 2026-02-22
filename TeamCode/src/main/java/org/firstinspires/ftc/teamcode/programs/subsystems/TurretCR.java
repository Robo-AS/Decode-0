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
    private final Limelight3A limelight;
    private final RTPAxon axon;

    public enum TurretState { GOAL_LOCK, LIMELIGHT_LOCK, MANUAL }
    public TurretState currentState = TurretState.GOAL_LOCK;
    public static double targetAngle = 0.0;

    public static double kP = 0.0125, kI = 0, kD = 0.0005, kS = 0.075;
    public static double MAX_ANGLE = 360.0, MIN_ANGLE = -90.0;
    public double downY = 0, upY = 1, maxDistance = 140, minDistance = 20;

    double angleToGoalField;
    private final TreeMap<Double, Double> yInterpolationTable = new TreeMap<>();

    public TurretCR() {
        this.limelight = robot.limelight;
        this.axon = robot.axon;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initialize() {
        axon.initialize(0);
        yInterpolationTable.put(0.0, 1.0);
        yInterpolationTable.put(136.0, 50.0);
    }

    @Override
    public void periodic() {
        applyToHardware();
    }

    private void updateGoalLock(boolean isRedAlliance, double robotX, double robotY) {
        double robotHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double goalY = isRedAlliance ? 150 : -8;
        double goalX = 136.0;

        if ((goalX - robotX) != 0)
            angleToGoalField = Math.atan((goalY - robotY) / (goalX - robotX));
        else
            angleToGoalField = 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetAngle = Math.toDegrees(diff);
    }

    public void loopAuto(boolean isRedAlliance, Pose pedro, double gX, double gY) {
        double robotHeading = pedro.getHeading() - Math.toRadians(90);
        double robotX = pedro.getX();
        double robotY = pedro.getY();

        double goalX = gX;
        double goalY = gY;

        if ((goalY - robotY) != 0)
            angleToGoalField = Math.atan((goalX - robotX) / (goalY - robotY));
        else
            angleToGoalField = 0;

        double diff = angleToGoalField + robotHeading;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        targetAngle = Math.toDegrees(diff);

        applyToHardware();
    }

    private void updateLimelight(int targetID) {
        LLResult ll = limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            boolean found = false;
            if (ll.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult ft : ll.getFiducialResults()) {
                    if (ft.getFiducialId() == targetID) { found = true; break; }
                }
            }
            if (found) {
                targetAngle = axon.getCurrentAngle() + ll.getTx();
            }
        }
    }

    public void loop(TurretState mode, int targetID, boolean isRed, double robotX, double robotY) {
        this.currentState = mode;
        if (currentState == TurretState.GOAL_LOCK) updateGoalLock(isRed, robotX, robotY);
        else if (currentState == TurretState.LIMELIGHT_LOCK) updateLimelight(targetID);
    }

    public void loopAuto(double target){
        targetAngle = target;
        applyToHardware();
    }

    private void applyToHardware() {
        double current = axon.getCurrentAngle();
        double target = targetAngle;

        while (target - current > 180) target -= 360;
        while (target - current < -180) target += 360;

        if (target < MIN_ANGLE) target += 360;
        if (target > MAX_ANGLE) target -= 360;

        targetAngle = Range.clip(target, MIN_ANGLE, MAX_ANGLE);

        axon.updatePIDCoeffs(kP, kI, kD, kS);
        axon.setTargetRotation(targetAngle);
        axon.update();
    }

    public double getAngleToGoalField() { return angleToGoalField; }
    public double getTargetAngle() { return targetAngle; }
}