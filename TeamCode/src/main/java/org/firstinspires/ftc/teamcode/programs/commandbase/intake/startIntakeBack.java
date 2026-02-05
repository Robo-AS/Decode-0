package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startIntakeBack extends InstantCommand {
    public startIntakeBack(double power) {
        super(
                () -> Robot.getInstance().intakeBack.setPower(power)
        );
    }
}