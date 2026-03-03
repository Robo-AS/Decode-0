package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import kotlin.time.Instant;

public class SetServoIntakeState extends InstantCommand {
    public SetServoIntakeState(Intake.ServoIntakeState servoIntakeState){
        super(
                () -> Robot.getInstance().intake.updateServoIntake(servoIntakeState)
        );
    }
}
