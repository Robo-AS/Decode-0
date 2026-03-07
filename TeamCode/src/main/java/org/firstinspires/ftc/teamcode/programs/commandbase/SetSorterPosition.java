package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class SetSorterPosition extends InstantCommand {
    public SetSorterPosition(double pos){
        super(
                () -> Robot.getInstance().servoSorter.setPosition(pos)
        );
    }
}
