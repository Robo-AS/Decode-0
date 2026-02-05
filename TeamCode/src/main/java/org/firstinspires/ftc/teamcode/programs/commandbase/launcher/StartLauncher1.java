package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class StartLauncher1 extends InstantCommand {
    public StartLauncher1(double pow){
        super(
                () -> Robot.getInstance().launcher1.setPower(pow)
        );
    }
}
