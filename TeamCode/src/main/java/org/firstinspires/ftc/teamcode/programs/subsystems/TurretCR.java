package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import java.util.TreeMap;

public class TurretCR extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final Limelight3A limelight;
    private final RTPAxon axon;

    public enum TurretState {
        GOAL_LOCK,
        LIMELIGHT_LOCK,
        MANUAL
    }

    public TurretState currentState = TurretState.GOAL_LOCK;
    public static double targetAngle = 0.0;

    public static double kP = 0.0125, kI = 0, kD = 0.0005, kS = 0.075;
    public static double MAX_ANGLE = 360.0, MIN_ANGLE = -90.0;

    private final TreeMap<Double, Double> goalAdjustmentLUT = new TreeMap<>();

    public TurretCR() {
        this.limelight = robot.limelight;
        this.axon = robot.axon;
    }

    public void initialize() {
        axon.initialize(0);
        setupLUT();
    }

    private void setupLUT() {
        goalAdjustmentLUT.put(-Math.PI/2, -4.0);
        goalAdjustmentLUT.put(-Math.PI/4, -2.0);
        goalAdjustmentLUT.put(0.0, 0.0);
        goalAdjustmentLUT.put(Math.PI/4, 2.0);
        goalAdjustmentLUT.put(Math.PI/2, 4.0);
    }

    private double getLUTAdjustment(double input) {
        Double lowKey = goalAdjustmentLUT.floorKey(input);
        Double highKey = goalAdjustmentLUT.ceilingKey(input);
        if (lowKey == null) return goalAdjustmentLUT.get(highKey);
        if (highKey == null || lowKey.equals(highKey)) return goalAdjustmentLUT.get(lowKey);
        return goalAdjustmentLUT.get(lowKey) + (input - lowKey) *
                (goalAdjustmentLUT.get(highKey) - goalAdjustmentLUT.get(lowKey)) / (highKey - lowKey);
    }

    private void updateGoalLock(boolean isRedAlliance) {
        if (robot.pinpoint == null) return;

        Pose2D pose = robot.pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double rawHeading = pose.getHeading(AngleUnit.RADIANS);

        double robotHeading = AngleUnit.normalizeRadians(-rawHeading + Math.PI / 2);

        double goalX = isRedAlliance ? 144.0 : 0.0;
        double goalY = 144.0;
        double velForward = robot.pinpoint.getVelX(DistanceUnit.INCH);
        double velStrafe  = robot.pinpoint.getVelY(DistanceUnit.INCH);

        double vx_field = velForward * Math.cos(robotHeading) - velStrafe * Math.sin(robotHeading);
        double vy_field = velForward * Math.sin(robotHeading) + velStrafe * Math.cos(robotHeading);

        double distToGoal = Math.hypot(goalX - robotX, goalY - robotY);
        double projectileSpeed_in_per_s = 130.0;

        if (distToGoal > 10.0 && Math.hypot(vx_field, vy_field) > 2.0) {
            double timeOfFlight = distToGoal / projectileSpeed_in_per_s;
            goalX += vx_field * timeOfFlight;
            goalY += vy_field * timeOfFlight;
        }

        double adjustment = getLUTAdjustment(robotHeading);

        double deltaX = (goalX + adjustment) - robotX;
        double deltaY = goalY - robotY;

        double angleToGoalField = Math.atan2(deltaY, deltaX);
        double relativeAngleRad = AngleUnit.normalizeRadians(angleToGoalField - robotHeading);

        targetAngle = -Math.toDegrees(relativeAngleRad);
    }

    private void updateLimelight(int targetID) {
        LLResult ll = limelight.getLatestResult();
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
                targetAngle = axon.getCurrentAngle() + ll.getTx();
            } else {
                targetAngle = 0.0;
            }
        } else {
            targetAngle = 0.0;
        }
    }

    public void loop(TurretState mode, int targetID, boolean isRed) {
        this.currentState = mode;
        switch (currentState) {
            case GOAL_LOCK: updateGoalLock(isRed); break;
            case LIMELIGHT_LOCK: updateLimelight(targetID); break;
            case MANUAL: break;
        }
        applyToHardware();
    }

    private void applyToHardware() {
        double normalized = AngleUnit.normalizeDegrees(targetAngle);
        if (normalized < -90.0) {
            normalized += 360.0;
        }
        targetAngle = Range.clip(normalized, MIN_ANGLE, MAX_ANGLE);

        axon.updatePIDCoeffs(kP, kI, kD, kS);
        axon.setTargetRotation(targetAngle);
        axon.update();
    }

    public void loopAuto(double target){
        targetAngle = target;
        applyToHardware();
    }

    @Override
    public void periodic() {}
}