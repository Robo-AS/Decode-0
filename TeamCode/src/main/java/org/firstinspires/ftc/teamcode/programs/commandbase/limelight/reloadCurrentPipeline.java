package org.firstinspires.ftc.teamcode.programs.commandbase.limelight;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class reloadCurrentPipeline extends InstantCommand {
    public reloadCurrentPipeline(){
        super(
                () -> Robot.getInstance().limelight.reloadPipeline()
        );
    }
}
