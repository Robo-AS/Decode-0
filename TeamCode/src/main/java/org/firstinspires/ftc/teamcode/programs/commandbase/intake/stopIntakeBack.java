package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class stopIntakeBack extends InstantCommand {
    public stopIntakeBack(){
        super(
                () -> Robot.getInstance().intakeBack.setPower(0)
        );
    }
}
