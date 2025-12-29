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

    public static double targetAngle = 0.0;
    public static double kP = 0.0125;
    public static double kI = 0.001;
    public static double kD = 0.000003;
    public static double MAX_ANGLE = 100.0;

    public TurretCR() {
        this.limelight = robot.limelight;
        this.axon = robot.axon;
    }

    public void initialize(){
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        axon.initialize();
    }

    private void updateHeadings(int targetID, boolean targetRedGoal) {
        LLResult ll = limelight.getLatestResult();
        boolean seesTargetID = false;

        if (ll != null && ll.isValid()) {
            for (LLResultTypes.FiducialResult apriltag : ll.getFiducialResults()) {
                if (apriltag.getFiducialId() == targetID) {
                    seesTargetID = true;
                    break;
                }
            }
        }

        if (ll != null && ll.isValid() && seesTargetID) {
            // tx and axon current angle are both degrees
            targetAngle = axon.getCurrentAngle() + ll.getTx();
        }
        else if (robot.pinpoint != null) {
            robot.pinpoint.update();
            Pose2D pose = robot.pinpoint.getPosition();

            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double robotHeading = pose.getHeading(AngleUnit.RADIANS);

            double goalX = targetRedGoal ? 144 : 0;
            double goalY = 144;

            double angleToGoalField = Math.atan2(goalY - robotY, goalX - robotX);
            double relativeAngleRad = AngleUnit.normalizeRadians(angleToGoalField - robotHeading);

            // Calculate the angle in degrees first
            double calculatedAngle = -Math.toDegrees(relativeAngleRad) + 90;

            // Your preferred if-else clamping logic in DEGREES
            if (calculatedAngle > MAX_ANGLE) {
                targetAngle = MAX_ANGLE;
            } else if (calculatedAngle < -MAX_ANGLE) {
                targetAngle = -MAX_ANGLE;
            } else {
                targetAngle = calculatedAngle;
            }
        }
        else {
            targetAngle = 0;
        }

        // Final safety check to ensure targetAngle is never NaN or out of bounds
        if (targetAngle > MAX_ANGLE) targetAngle = MAX_ANGLE;
        if (targetAngle < -MAX_ANGLE) targetAngle = -MAX_ANGLE;
    }

    public void loop(int targetID, boolean targetRedGoal) {
        updateHeadings(targetID, targetRedGoal);
        axon.updatePIDCoeffs(kP, kI, kD);
        axon.setTargetRotation(targetAngle);
        axon.update();
    }

    public void loopAuto(double target){
        if (target > MAX_ANGLE) targetAngle = MAX_ANGLE;
        else if (target < -MAX_ANGLE) targetAngle = -MAX_ANGLE;
        else targetAngle = target;

        axon.updatePIDCoeffs(kP, kI, kD);
        axon.setTargetRotation(targetAngle);
        axon.update();
    }
}