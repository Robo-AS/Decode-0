package org.firstinspires.ftc.teamcode.programs.commandbase.limelight;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class switchToArtifactPipeline extends InstantCommand {
    public switchToArtifactPipeline(){
        super(
                () -> Robot.getInstance().getInstanceLimelight().useArtifactPipeline()
        );
    }
}