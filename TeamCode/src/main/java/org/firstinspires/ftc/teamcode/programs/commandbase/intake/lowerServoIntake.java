package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;


public class lowerServoIntake extends InstantCommand {
    public lowerServoIntake(){
        super(
                () -> Robot.getInstance().servoIntake.setPosition(0.75)
        );
    }
}
