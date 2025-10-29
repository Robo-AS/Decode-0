package org.firstinspires.ftc.teamcode.programs.commandbase.limelight;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;


public class setServoYPosition extends InstantCommand {
    public setServoYPosition(double angle){
        super(
                () -> Robot.getInstance().servoY.setPosition(angle)
        );
    }
}
