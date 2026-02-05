package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startIntakeFront extends InstantCommand {
    public startIntakeFront(double power) {
        super(
                () -> Robot.getInstance().intakeFront.setPower(power)
        );
    }
}