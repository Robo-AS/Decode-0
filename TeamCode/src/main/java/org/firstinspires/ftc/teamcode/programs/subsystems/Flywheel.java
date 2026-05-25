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
    public double previousBarrier = 0.0, previousLauncher = 0.0, targetBarrier = 0.35;

    public enum BarrierState{
        FREE,
        BLOCK
    }

    public static double FREE = 0.5;
    public static double BLOCK = 0.35;

    public BarrierState barrierState;

    public void updateBarrierState(BarrierState state){
        barrierState = state;

        switch (barrierState){
            case FREE:
                targetBarrier = FREE;
                break;
            case BLOCK:
                targetBarrier = BLOCK;
                break;
        }
    }

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



        if(targetBarrier != previousBarrier)
            robot.servoBarrier.setPosition(targetBarrier);

        previousBarrier = targetBarrier;
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

        if(targetBarrier != previousBarrier) {
            robot.servoBarrier.setPosition(targetBarrier);
        }
        previousBarrier = targetBarrier;
    }

    public double getCurrentVelocity(){
        return currentVelocity;
    }

    public double getTargetVelocity() { return targetVelocity; }

    public void initializeVelInterpLUT() {
        vel = new InterpLUT();

        vel.add(-300.0000, 1600);
        vel.add(0.0000, 1600);
        vel.add(40.8133, 1600);
        vel.add(47.4500, 1600);
        vel.add(57.6200, 1650);
        vel.add(67.8110, 1750);
        vel.add(81.0053, 1950);
        vel.add(98.2378, 2000);
        vel.add(115.2377, 2265);
        vel.add(123.7950, 2360);
        vel.add(130.9599, 2300);
        vel.add(140.1241, 2570);
        vel.add(10000, 2750);
        vel.createLUT();
    }
}
