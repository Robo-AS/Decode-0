package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public DcMotorEx intakeMotor;
    double motorPower = 1;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void teleop(GamepadEx gamepad, Telemetry telemetry)
    {
        if (gamepad.isDown(GamepadKeys.Button.Y))
        {
            intakeMotor.setPower(motorPower);
        }
        else
        {
            intakeMotor.setPower(0);
        }
    }

}
