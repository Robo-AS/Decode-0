package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class freeServoBarrier extends InstantCommand {
    public freeServoBarrier()
    {
        super(
                () -> Robot.getInstance().servoBarrier.setPosition(0.525)
        );
    }
}
