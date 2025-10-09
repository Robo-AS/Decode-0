package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startRightLauncher extends InstantCommand {
    public startRightLauncher(double power) {
        super(
                () -> Robot.getInstance().rightLauncher.setPower(power)
        );
    }
}