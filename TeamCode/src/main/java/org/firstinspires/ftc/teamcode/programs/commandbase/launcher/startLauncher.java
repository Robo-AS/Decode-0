package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class startLauncher extends InstantCommand {
    public startLauncher(double joystickPower) {
        super(
                () -> Robot.getInstance().launcher1.setPower(joystickPower)
        );
    }
}