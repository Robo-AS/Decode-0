package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class Shoot extends SequentialCommandGroup {
    public Shoot(){
        super(
                new startLauncher(1),
                new WaitCommand(2000),
                new setServoLauncherPosition(1)
        );
    }
}
