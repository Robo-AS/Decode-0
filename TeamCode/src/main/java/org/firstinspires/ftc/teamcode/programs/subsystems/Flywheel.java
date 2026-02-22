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
        currentVelocity = flyWheel1.getVelocity();
        targetVelocity = vel.get(distance);

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

        vel.add(0.0000, 1600);
        vel.add(55.8133, 1600);
        vel.add(69.4260, 1675);
        vel.add(76.2866, 1675);
        vel.add(90.9241, 1800); //1750
        vel.add(102.6444, 1850);
        vel.add(118.0427, 1950);
        vel.add(128.0427, 1950);
        vel.add(133.0427, 1950);
        vel.add(141.8654, 2300);
        vel.add(300, 2300);

        vel.createLUT();
    }
}

/*
55.8133 1600
69.4260 1800
76.2866 1800
90.9241 1850
102.6444 1900
118.0427 2100
130 2300
141.8654 2350
 */