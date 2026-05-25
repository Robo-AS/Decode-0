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

    public void loop(double distance) {
        this.distance = distance;

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
        hd.add(40.8133, 0);
        hd.add(47.4500, 0.1);
        hd.add(57.6200, 0.35);
        hd.add(67.8110, 0.45);
        hd.add(83.0053, 0.7);
        hd.add(98.2378, 0.8);
        hd.add(115.0200, 0.915);
        hd.add(123.2991, 0.93);
        hd.add(130.8019, 0.985);
        hd.add(140.1241, 1);
        hd.add(10000, 1);

        hd.createLUT();
    }
}