package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startLauncher extends SequentialCommandGroup {
    public startLauncher() {
        new startLeftLauncher(0.4);
        new startRightLauncher(0.4);
        new WaitCommand(2000);
        new startLeftLauncher(0);
        new startRightLauncher(0);
    }
}