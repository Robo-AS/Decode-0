package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final DcMotorEx front;
    private final DcMotorEx back;
    private final Servo intakeServo;
    private boolean isFull = false;

    private double targetPower = 0.0, targetPosition = 0.0, previousPower = 0.0, previousPosition = 0.0, previousBarrierPosition = 0.0;

    public Intake() {
        this.front = robot.intakeFront;
        this.back = robot.intakeBack;
        this.intakeServo = robot.servoIntake;
    }

    public enum IntakeState{
        ON,
        OFF,
        REVERSED_ON
    }

    public enum ServoIntakeState{
        LIFT,
        LOWER
    }

    public enum BarrierState{
        FREE,
        BLOCk
    }

    public IntakeState intakeState;
    public ServoIntakeState servoIntakeState;

    public static int ON = 1;
    public static int OFF = 0;
    public static int REVERSED_ON = -1;
    public static int LIFT = 0;
    public static double LOWER = 0.3;


    public void updateIntake(IntakeState state){
        intakeState = state;

        switch (intakeState){
            case ON:
                targetPower = ON;
                break;
            case OFF:
                targetPower = OFF;
                break;
            case REVERSED_ON:
                targetPower = REVERSED_ON;
                break;
        }
    }

    public void updateServoIntake(ServoIntakeState state){
        servoIntakeState = state;

        switch (servoIntakeState){
            case LIFT:
                targetPosition = LIFT;
                break;
            case LOWER:
                targetPosition = LOWER;
                break;
        }
    }

    public void loop(){
        if(targetPosition != previousPosition)
            intakeServo.setPosition(targetPosition);

        if(targetPower != previousPower) {
            front.setPower(targetPower);
            back.setPower(-targetPower);
        }

        previousPosition = targetPosition;
        previousPower = targetPower;
    }
}