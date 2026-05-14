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
        hd.add(55.8133, 0);
        hd.add(62.4500, 0.1);
        hd.add(72.6200, 0.35);
        hd.add(82.8110, 0.45);
        hd.add(96.0053, 0.7);
        hd.add(113.2378, 0.8);
        hd.add(136.0200, 0.915);
        hd.add(145.2991, 0.93);
        hd.add(148.8019, 0.985);
        hd.add(162.1241, 1);
        hd.add(10000, 1);

        hd.createLUT();
    }
}