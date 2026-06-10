package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;
import com.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Hood extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    // Valorile acestea pot rămâne ca referință, dar LUT-ul le va suprascrie
    public double upY = 0.85;
    public double distance;
    public InterpLUT hd = new InterpLUT();
    public HoodServoState hoodServoState = HoodServoState.AUTOMATED;

    public enum HoodServoState {
        AUTOMATED,
        FAR_ZONE
    }

    public void initialize() {
        initializeHoodInterpLUT();

        hoodServoState = HoodServoState.AUTOMATED;
        distance = 144;
    }

    public void loop(double distance, double angleToGoal) {
        this.distance = distance;
        double velX=-Robot.getInstance().turret.getRobotVx();
        double velY=Robot.getInstance().turret.getRobotVy();
        double robotVelocityAngle = Math.atan2(velY, velX);
        double robotVelocityMagnitude = Math.hypot(velX, velY);
        double alpha=Math.abs (angleToGoal-robotVelocityAngle);
        double finalMagnitude=Math.sqrt (robotVelocityMagnitude*robotVelocityMagnitude+distance*distance+2*robotVelocityMagnitude*distance*Math.cos (alpha));

        if (hoodServoState == HoodServoState.AUTOMATED) {
            double targetPosition = hd.get(distance);
            robot.hoodServo.setPosition(targetPosition);
        }
    }

    public void updateHoodServoState(HoodServoState state) {
        this.hoodServoState = state;
        switch (hoodServoState) {
            case AUTOMATED:
                break;
            case FAR_ZONE:
                robot.hoodServo.setPosition(upY);
                break;
        }
    }

    @Deprecated
    public double calculateHoodAngle(double distance) {
        return hd.get(distance);
    }

    public void initializeHoodInterpLUT() {
        hd = new InterpLUT();

        hd.add(-300.0000, 0);
        hd.add(0.0000, 0);
        hd.add(44.0357 , 0);
        hd.add(58.7474, 0.15);
        hd.add(72.5890, 0.4);
        hd.add(94.1681, 0.7);
        hd.add(110.6160, 0.75);
        hd.add(129.8455, 0.89);
        hd.add(140.4053, 0.925);
        hd.add(146.3112, 0.9);
        hd.add(156.7391, 0.94);
        hd.add(10000, 1);

        hd.createLUT();
    }
}