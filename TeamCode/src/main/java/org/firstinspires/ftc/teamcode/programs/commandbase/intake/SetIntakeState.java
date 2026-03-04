package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;


public class SetIntakeState extends InstantCommand {
    public SetIntakeState(Intake.IntakeState intakeState){
        super(
                () -> Robot.getInstance().intake.updateIntakeMotor(intakeState)
        );
    }
}
