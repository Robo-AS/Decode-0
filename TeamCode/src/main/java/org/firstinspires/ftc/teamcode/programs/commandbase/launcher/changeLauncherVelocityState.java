package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class changeLauncherVelocityState extends InstantCommand {
    public changeLauncherVelocityState(boolean setState){
        super(
                () -> Robot.getInstance().shootFar = setState
        );
    }
}
