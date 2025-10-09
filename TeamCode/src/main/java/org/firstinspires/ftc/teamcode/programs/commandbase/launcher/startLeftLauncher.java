package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startLeftLauncher extends InstantCommand {
    public startLeftLauncher(double power) {
        super(
                () -> Robot.getInstance().leftLauncher.setPower(power)
        );
    }
}