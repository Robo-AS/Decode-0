package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class changeAimState extends InstantCommand {
    public changeAimState(boolean aimState){
        super(
                () -> Robot.getInstance().limelightOnlyAim = aimState
        );
    }
}
