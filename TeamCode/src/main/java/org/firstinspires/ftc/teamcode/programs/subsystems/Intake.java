package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum IntakeState{
        ON,
        OFF,
        REVERSED_ON
    }

    public enum ServoIntakeState{
        UP,
        DOWN
    }


    public IntakeState intakeState;
    public ServoIntakeState servoIntakeState;
    public static int UP = 0;
    public static double DOWN = 0.3;


    public void updateIntakeMotor(IntakeState state){
        intakeState = state;

        switch (intakeState){
            case ON:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(-1);
                break;
            case OFF:
                robot.intakeFront.setPower(0);
                robot.intakeBack.setPower(0);
                break;
            case REVERSED_ON:
                robot.intakeFront.setPower(-1);
                robot.intakeBack.setPower(0);
                break;
        }
    }

    public void updateIntakeServo(ServoIntakeState state){
        servoIntakeState = state;

        switch (servoIntakeState){
            case UP:
                robot.servoIntake.setPosition(UP);
                break;
            case DOWN:
                robot.servoIntake.setPosition(DOWN);
                break;
        }
    }

//    public void loop(){
//        if(targetPosition != previousPosition)
//            intakeServo.setPosition(targetPosition);
//
//        if(targetPower != previousPower) {
//            front.setPower(targetPower);
//            back.setPower(-targetPower);
//        }
//
//        previousPosition = targetPosition;
//        previousPower = targetPower;
//    }
}