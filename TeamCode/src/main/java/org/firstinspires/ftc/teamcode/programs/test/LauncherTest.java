package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.startLauncher;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Launcher Test", group = "OpModes")
public class LauncherTest extends CommandOpMode {
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

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        new RunCommand(
                                () -> Robot.getInstance().launcher1.setPower(Math.abs(gamepadEx.getLeftY()))
                        )
                )
                .whenReleased(
                        new RunCommand(
                                () -> Robot.getInstance().launcher1.setPower(Math.abs(0))
                        )
                );


    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        Robot.getInstance().getInstanceLimelight().loop();
    }
}