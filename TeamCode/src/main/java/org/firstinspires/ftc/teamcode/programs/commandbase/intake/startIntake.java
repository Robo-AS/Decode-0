package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startIntake extends InstantCommand {
    public startIntake(){
        super(
                () -> Robot.getInstance().intake.setPower(0.4)
        );
    }
}