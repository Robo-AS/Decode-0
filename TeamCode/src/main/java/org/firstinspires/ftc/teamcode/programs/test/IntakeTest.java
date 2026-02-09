package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeBack;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeFront;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeBack;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeFront;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Intake Test", group = "OpModes")
public class IntakeTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);
        robot.initialize();


        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new SequentialCommandGroup(
                                new startIntakeFront(1),
                                new startIntakeBack(1),
                                new WaitCommand(1500),
                                new stopIntakeFront(),
                                new stopIntakeBack()
                        )
                )
                .whenReleased(
                        new stopIntakeFront()
                );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

    }
}