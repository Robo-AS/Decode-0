package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import java.util.List;

public class Turret extends SubsystemBase {
    private Servo servoX, servoY;
    private Limelight3A limelight;
    private final double centerX = 0.5;
    private final double centerY = 0.5;
    private final double upY = 0.75;
    private final double downY = 0.25;
    private final double yawMaxDeg = 150.0;
    private final double pitchMaxDeg = 20.0;
    private final double cameraHeightM = 0.2925;
    private final double limelightAngleMounted = 25.0; // in grade
    private final double limelightLensHeight = 0.2925; //in metri
    private final double goalHeightMeters = 0.784; //in metri
    private final double gearRatio = 1.5;
    private double servoXPos = centerX;
    private boolean scanningRight = true;
    private final double scanSpeed = 0.01;
    private final int targetID = 20;

    public void initialize() {
        Robot robot = Robot.getInstance();

        servoX = robot.servoX;
        servoY = robot.servoY;
        limelight = robot.limelight;

        servoX.setPosition(centerX);
        servoY.setPosition(centerY);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid())
        {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if(tags != null && !tags.isEmpty())
            {
                LLResultTypes.FiducialResult targetTag = null;

                for(LLResultTypes.FiducialResult tag : tags)
                {
                    if(tag.getFiducialId() == targetID)
                    {
                        targetTag = tag;
                        break;
                    }
                }

                if(targetTag != null)
                {
                    double distance = getDistanceFromTag(targetTag);

                    if (distance == -1)
                        return;

                    double targetYaw = getYawFromTag(targetTag);
                    double targetPitch = getPitchFromDistance(distance);

                    servoXPos = centerX + ((targetYaw * gearRatio) / Math.toRadians(yawMaxDeg)) * 0.5;
                    servoXPos = clamp(servoXPos, 0.0, 1.0);
                    servoX.setPosition(servoXPos);

                    double servoYPos = centerY + (targetPitch / pitchMaxDeg) * (upY - centerY);
                    servoYPos = clamp(servoYPos, downY, upY);
                    servoY.setPosition(servoYPos);

                    return;
                }
            }
            scanWhenNoTarget();
        }
        else
        {
            scanWhenNoTarget();
        }
    }

    private void scanWhenNoTarget()
    {
        if(scanningRight)
        {
            servoXPos += scanSpeed;
            if(servoXPos >= 1.0)
                scanningRight = false;
        }
        else
        {
            servoXPos -= scanSpeed;
            if(servoXPos <= 0.0)
                scanningRight = true;
        }

        servoXPos = clamp(servoXPos, 0.0, 1.0);
        servoX.setPosition(servoXPos);
    }

    private double getDistanceFromTag(LLResultTypes.FiducialResult tag)
    {
        double ty = tag.getTargetYDegreesNoCrosshair();
        double angleToGoalDegrees = limelightAngleMounted + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (goalHeightMeters - limelightLensHeight) / Math.tan(angleToGoalRadians);
    }

    private double getYawFromTag(LLResultTypes.FiducialResult tag)
    {
        double tx = tag.getTargetXDegreesNoCrosshair();
        double yawRad = Math.toRadians(tx);

        return clamp(yawRad, -Math.toRadians(yawMaxDeg), Math.toRadians(yawMaxDeg));
    }

    private double getPitchFromDistance(double distanceMeters)
    {
        double dz = goalHeightMeters - cameraHeightM;
        double pitchRad = Math.atan2(dz, distanceMeters);
        double pitchDeg = Math.toDegrees(pitchRad);

        return clamp(pitchDeg, -pitchMaxDeg, pitchMaxDeg);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
