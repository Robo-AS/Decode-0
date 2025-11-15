package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import com.solverslib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Flywheel extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0.00001;
    public static double kS = 0.0435;
    public static double kV = 0.000275;

    public InterpLUT vel = new InterpLUT();

    private DcMotorEx flyWheel1, flyWheel2;
    PIDFController pid_Flywheel;
    SimpleMotorFeedforward feedforward;

    public static double targetVelocity = 0, currentVelocity = 0;

    public void initialize() {
        flyWheel1 = robot.launcher1;
        flyWheel2 = robot.launcher2;

        pid_Flywheel = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        initializeVelInterpLUT();
    }

    public void loop(double distance) {
        currentVelocity = flyWheel1.getVelocity();
        targetVelocity = vel.get(distance);

        pid_Flywheel.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        double ff = feedforward.calculate(targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);
        double power = pid + ff;

        flyWheel1.setPower(power);
        flyWheel2.setPower(power);
    }

    public void initializeVelInterpLUT()
    {
        vel = new InterpLUT();
        vel.add(0, 3100);
        vel.add(0.0270, 3100);
        vel.add(0.0454, 2800);
        vel.add(0.0562, 1950);
        vel.add(0.0668, 1900);
        vel.add(0.0900, 1700);
        vel.add(0.1820, 1500);
        vel.add(0.2704, 900);
        vel.createLUT();

    }

}

//0.027 3100
//0.0668 1900
//0.09 1700
//0.182 1500
//0.0562 1950
//0.0454 2800
