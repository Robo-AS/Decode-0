package org.firstinspires.ftc.teamcode.programs.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class StrafeMecanumToArtifact extends InstantCommand {
    public StrafeMecanumToArtifact() {
        super(
                () -> Robot.getInstance().mecanum.strafePID(Robot.getInstance().cameraPipeline.currentX)
        );
    }
}