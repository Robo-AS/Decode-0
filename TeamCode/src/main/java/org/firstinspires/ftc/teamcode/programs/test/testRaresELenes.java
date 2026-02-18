package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

@TeleOp(name = "Se invarta tot si sa ma suga Rares", group = "OpModes")
public class testRaresELenes extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new ParallelCommandGroup(
                                new startIntakeFront(1),
                                new startIntakeBack(1),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher1.setPower(1)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher2.setPower(1)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().servoBarrier.setPosition(1)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().servoY.setPosition(1)
                                )
                        )
                )
                .whenReleased(
                        new ParallelCommandGroup(
                                new startIntakeFront(0),
                                new startIntakeBack(0),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher1.setPower(0)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().launcher2.setPower(0)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().servoBarrier.setPosition(0.5)
                                ),
                                new RunCommand(
                                        () -> Robot.getInstance().servoY.setPosition(0)
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