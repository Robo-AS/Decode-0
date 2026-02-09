package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class changeAimState extends InstantCommand {
    public changeAimState(){
        super(
                () -> Robot.getInstance().limelightAimOnly = !Robot.getInstance().limelightAimOnly
        );
    }
}
