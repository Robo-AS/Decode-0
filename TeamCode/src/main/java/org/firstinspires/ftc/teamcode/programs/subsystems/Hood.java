package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Hood extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public double downY = 0, upY = 0.85, maxDistance = 140, minDistance = 20, distance;
    public HoodServoState hoodServoState = HoodServoState.AUTOMATED;


    public enum HoodServoState{
        AUTOMATED,
        FAR_ZONE
    }

    public void initialize(){
        hoodServoState = HoodServoState.AUTOMATED;
        distance = 144;
    }

    public void loop(double distance){
        calculateHoodAngle(distance);
        if(hoodServoState == HoodServoState.AUTOMATED)
            robot.hoodServo.setPosition(calculateHoodAngle(distance));
    }

    public void updateHoodServoState(HoodServoState state){
        hoodServoState = state;
        switch (hoodServoState) {
            case AUTOMATED:
                break;
            case FAR_ZONE:
                robot.hoodServo.setPosition(0.85);
                break;
        }
    }


    public double calculateHoodAngle(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);
        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);
        return downY + ratio * (upY - downY);
    }
}
