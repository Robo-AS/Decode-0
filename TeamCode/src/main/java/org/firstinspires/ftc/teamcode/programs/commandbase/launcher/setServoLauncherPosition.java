package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class setServoLauncherPosition extends InstantCommand {
    public setServoLauncherPosition(double pos){
            super(
                    () -> Robot.getInstance().servoLauncher.setPosition(pos)
            );
    }
}
