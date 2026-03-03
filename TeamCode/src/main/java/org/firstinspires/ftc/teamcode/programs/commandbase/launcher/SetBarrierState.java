package org.firstinspires.ftc.teamcode.programs.commandbase.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class SetBarrierState extends InstantCommand {
    public SetBarrierState(Flywheel.BarrierState barrierState){
        super(
                () -> Robot.getInstance().flywheel.updateBarrierState(barrierState)
        );
    }
}
