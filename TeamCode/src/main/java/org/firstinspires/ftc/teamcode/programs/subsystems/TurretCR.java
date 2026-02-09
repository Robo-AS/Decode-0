package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class TurretCR extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    private final Limelight3A limelight;
    private RTPAxon axon;

    public static double targetAngle = 0.0, lastValidTx = 0.0;
    private boolean wasSeeingTarget = false;

    public static double kP = 0.0125525225005054325;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kS = 0.075;
    public static double MAX_ANGLE = 90, MIN_ANGLE = -360;
    public static double MAX_TX_JUMP = 3.0;

    public TurretCR() {
        this.limelight = robot.limelight;
        this.axon = robot.axon;
    }

    public void initialize() {
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        axon.initialize(0);
    }

    private void updateHeadings(int targetID, boolean targetRedGoal) {
        LLResult ll = limelight.getLatestResult();
        boolean seesTargetID = false;

        if (ll != null && ll.isValid() && !ll.getFiducialResults().isEmpty()) {
            for (LLResultTypes.FiducialResult apriltag : ll.getFiducialResults()) {
                if (apriltag.getFiducialId() == targetID) {
                    seesTargetID = true;
                    break;
                }
            }
        }

        if (ll != null && ll.isValid() && seesTargetID) {
            double currentTx = ll.getTx();

            if (!wasSeeingTarget) {
                lastValidTx = currentTx;
                wasSeeingTarget = true;
            } else if (Math.abs(currentTx - lastValidTx) < MAX_TX_JUMP) {
                lastValidTx = currentTx;
            }

            targetAngle = axon.getCurrentAngle() - lastValidTx;

        } else if (robot.pinpoint != null) {
            wasSeeingTarget = false;
            Pose2D pose = robot.pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double robotHeading = pose.getHeading(AngleUnit.RADIANS);

            double goalX = targetRedGoal ? 144.0 : 0.0;
            double goalY = 144.0;

            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;

            double angleToGoalField = Math.atan2(deltaY, deltaX);
            double relativeAngleRad = AngleUnit.normalizeRadians(angleToGoalField - robotHeading);
            double relativeDegrees = Math.toDegrees(relativeAngleRad);

            double TURRET_MOUNTING_OFFSET;

            if (robotX >= 70) {
                if (robotY >= 10 && robotY < 72) {
                    TURRET_MOUNTING_OFFSET = -53.0;
                    double increaseValue = 1 + (robotY - 10) * 0.0645;
                    TURRET_MOUNTING_OFFSET -= increaseValue;
                } else {
                    TURRET_MOUNTING_OFFSET = -55.0;
                    double decreaseValue = 5 + (robotY - 72) * ((1.0 - 5.0) / (110.0 - 72.0));
                    TURRET_MOUNTING_OFFSET += decreaseValue;
                }
            } else {
                TURRET_MOUNTING_OFFSET = -73.0;
                if (robotY >= 10 && robotY < 72) {
                    double pullLeftValue = (robotY - 10) * 0.08;
                    TURRET_MOUNTING_OFFSET -= pullLeftValue;
                } else {
                    double secondaryPullLeft = 5 + (robotY - 72) * 0.12;
                    TURRET_MOUNTING_OFFSET -= secondaryPullLeft;
                }
            }

            if (Math.abs(relativeDegrees + TURRET_MOUNTING_OFFSET - targetAngle) > 0.5)
                targetAngle = relativeDegrees + TURRET_MOUNTING_OFFSET;
        }

        double normalized = AngleUnit.normalizeDegrees(targetAngle);
        if (normalized > 90) normalized -= 360;
        targetAngle = Math.max(MIN_ANGLE, Math.min(normalized, MAX_ANGLE));
    }

    public void loop(int targetID, boolean targetRedGoal) {
        updateHeadings(targetID, targetRedGoal);
        applyToHardware();
    }

    public void loopAuto(double target) {
        double normalized = AngleUnit.normalizeDegrees(target);
        if (normalized > 90) normalized -= 360;
        targetAngle = Math.max(MIN_ANGLE, Math.min(normalized, MAX_ANGLE));
        applyToHardware();
    }

    public void loopLimelight(int targetID) {
        LLResult ll = limelight.getLatestResult();
        boolean seesTargetID = false;

        if (ll != null && ll.isValid() && !ll.getFiducialResults().isEmpty()) {
            for (LLResultTypes.FiducialResult apriltag : ll.getFiducialResults()) {
                if (apriltag.getFiducialId() == targetID) {
                    seesTargetID = true;
                    break;
                }
            }
        }

        if (ll != null && ll.isValid() && seesTargetID) {
            targetAngle = axon.getCurrentAngle() - ll.getTx();
        }

        targetAngle = Math.max(MIN_ANGLE, Math.min(targetAngle, MAX_ANGLE));
        applyToHardware();
    }

    private void applyToHardware() {
        axon.updatePIDCoeffs(kP, kI, kD, kS);
        axon.setTargetRotation(targetAngle);
        axon.update();
    }
}