package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;


public class increaseDriverOffset extends InstantCommand {
    public increaseDriverOffset(){
        super(
                () -> Robot.getInstance().driverOffset += 2.0
        );
    }
}
