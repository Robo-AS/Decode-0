package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startIntake extends InstantCommand {
    public startIntake(double joystickPower) {
        super(
                () -> Robot.getInstance().intake.setPower(joystickPower)
        );
    }
}