package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class blockServoBarrier extends InstantCommand {
    public blockServoBarrier()
    {
        super(
                () -> Robot.getInstance().servoBarrier.setPosition(0.35)
        );
    }
}
