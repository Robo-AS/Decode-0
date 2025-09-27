package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Drive", group = "OpModes")
public class firstTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap, (MultipleTelemetry) telemetry);
        robot.initialize();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double forward = -gamepadEx.getLeftY();
        double strafe = gamepadEx.getLeftX();
        double turn = gamepadEx.getRightX();

        robot.pinpoint.update();
        robot.mecanum.drive(strafe, forward, turn);
        robot.limelight.loop();
        robot.turret.loop(20);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new startIntake(),
                        new WaitCommand(2000),
                        new stopIntake()
                )
        );
    }
}