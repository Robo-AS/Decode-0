package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class stopIntakeFront extends InstantCommand {
    public stopIntakeFront(){
        super(
                () -> Robot.getInstance().intakeFront.setPower(0)
        );
    }
}