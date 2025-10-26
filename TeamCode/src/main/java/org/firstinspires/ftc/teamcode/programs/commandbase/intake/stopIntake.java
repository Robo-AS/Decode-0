package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class stopIntake extends InstantCommand {
    public stopIntake(){
        super(
                () -> Robot.getInstance().intake.setPower(0)
        );
    }
}