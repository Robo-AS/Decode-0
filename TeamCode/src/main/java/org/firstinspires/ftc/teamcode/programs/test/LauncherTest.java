package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Launcher Test", group = "OpModes")
public class LauncherTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        new ParallelCommandGroup(
                            new RunCommand(
                                    () -> Robot.getInstance().launcher1.setPower(Math.abs(gamepadEx.getLeftY()))
                            ),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher2.setPower(Math.abs(gamepadEx.getLeftY()))
                                )
                        )
                )
                .whenReleased(
                        new ParallelCommandGroup(
                                new RunCommand(
                                        () -> Robot.getInstance().launcher1.setPower(Math.abs(0))
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher2.setPower(Math.abs(0))
                                )
                        )
                );


    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

       // Robot.getInstance().getInstanceLimelight().loop();
    }
}