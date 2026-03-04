package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Hood;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;


public class SetHoodServoState extends InstantCommand {
    public SetHoodServoState(Hood.HoodServoState state){
        super(
                () -> Robot.getInstance().hood.updateHoodServoState(state)
        );
    }
}
