package org.firstinspires.ftc.teamcode.programs.commandbase.limelight;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class switchToAprilTagPipeline extends InstantCommand {
    public switchToAprilTagPipeline(){
        super(
                () -> Robot.getInstance().getInstanceLimelight().useAprilTagPipeline()
        );
    }
}