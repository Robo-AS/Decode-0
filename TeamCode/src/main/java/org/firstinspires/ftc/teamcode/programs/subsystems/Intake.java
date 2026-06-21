package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final static double JAM_CURRENT_THRESHOLD = 7.5;

    public enum IntakeState{
        ON,
        OFF,
        REVERSED_ON,
        AUTO_REVERSED_ON
    }

    public enum ServoIntakeState{
        UP,
        DOWN,
        AUTO_GATE
    }


    public IntakeState intakeState;
    public ServoIntakeState servoIntakeState;
    public static int UP = 0;
    public static double DOWN = 0.175;
    public static double AUTO_GATE = 0.1;


    public void updateIntakeMotor(IntakeState state){
        intakeState = state;

        switch (intakeState){
            case ON:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                break;
            case OFF:
                robot.intakeFront.setPower(0);
                robot.intakeBack.setPower(0);
                break;
            case REVERSED_ON:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(0);
                break;
            case AUTO_REVERSED_ON:
                robot.intakeFront.setPower(-1);
                robot.intakeBack.setPower(-1);
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
            case AUTO_GATE:
                robot.servoIntake.setPosition(AUTO_GATE);
                break;
        }
    }

    public boolean isJammed() {
        return robot.intakeBack.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) > JAM_CURRENT_THRESHOLD;
    }
}