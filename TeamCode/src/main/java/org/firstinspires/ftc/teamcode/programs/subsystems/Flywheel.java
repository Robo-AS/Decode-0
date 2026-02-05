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
    public static double kP = 0.0075;
    public static double kI = 0;
    public static double kD = 0.0000025;
    public static double kS = 0.07;
    public static double kV = 0.00025;

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

    public void loopAuto(double velocity){
        currentVelocity = flyWheel1.getVelocity();
        targetVelocity = velocity;

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
        vel.add(0, 3300);
        vel.add(0.01701, 3300);
        vel.add(0.022, 3300);
        vel.add(0.0356, 2450);
        vel.add(0.0466, 2200);
        vel.add(0.0627, 2000);
        vel.add(0.08, 1700);
        vel.add(0.1028, 1600);
        vel.add(0.1429, 1600);
        vel.add(0.206, 1600);
        vel.add(0.3, 1600);
        vel.add(0.4, 1600);
        vel.createLUT();
    }

}

/*
        WORKING

        vel = new InterpLUT();
        vel.add(0, 3300);
        vel.add(0.01701, 3300);
        vel.add(0.022, 3300);
        vel.add(0.0356, 2450);
        vel.add(0.0466, 2100);
        vel.add(0.0627, 1900);
        vel.add(0.08, 1600);
        vel.add(0.1028, 1600);
        vel.add(0.1429, 1500);
        vel.add(0.206, 1600);
        vel.add(0.3, 1600);
        vel.add(0.4, 1600);
        vel.createLUT();*/
