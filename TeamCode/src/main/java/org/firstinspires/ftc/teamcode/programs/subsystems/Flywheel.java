package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.solverslib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Flywheel extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0.05;
    public static double kV = 0.0003;

    private double smoothedDistance = 0.0627;
    private final double LOOKUP_FILTER = 0.1;

    public InterpLUT vel = new InterpLUT();
    private DcMotorEx flyWheel1, flyWheel2;
    private PIDFController pid_Flywheel;
    private SimpleMotorFeedforward feedforward;

    public static double targetVelocity = 0, currentVelocity = 0;

    public void initialize() {
        flyWheel1 = robot.launcher1;
        flyWheel2 = robot.launcher2;
        pid_Flywheel = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);
        initializeVelInterpLUT();
    }

    public void loop(double distance) {
        smoothedDistance = (LOOKUP_FILTER * distance) + ((1 - LOOKUP_FILTER) * smoothedDistance);

        currentVelocity = flyWheel1.getVelocity();
        targetVelocity = vel.get(smoothedDistance);

        pid_Flywheel.setPIDF(kP, kI, kD, 0);

        double ff = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);

        double power = Math.max(0, pid + ff);

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

    public double getCurrentVelocity(){
        return currentVelocity;
    }

    public void initializeVelInterpLUT() {
        vel = new InterpLUT();


        vel.add(0.0000, 2500);
        vel.add(0.0045, 2500);
        vel.add(0.0049, 2400);
        vel.add(0.0112, 2150);
        vel.add(0.0182, 1900);
        vel.add(0.0233, 1800);
        vel.add(0.0414, 1800);
        vel.add(0.0655, 1700);
        vel.add(0.1105, 1700);
        vel.add(0.1262, 1700);
        vel.add(0.5000, 1700);
        vel.add(1, 1700);
        vel.add(2, 1700);
        vel.add(20, 1700);

        vel.createLUT();
    }
}