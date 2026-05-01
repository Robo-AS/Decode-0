package org.firstinspires.ftc.teamcode.programs.commandbase.mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class SetLockMecanumServoState extends InstantCommand {
    public SetLockMecanumServoState(Mecanum.LockMecanumState state){
        super(
                () -> Robot.getInstance().mecanum.updateLockMecanumState(state)
        );
    }
}
