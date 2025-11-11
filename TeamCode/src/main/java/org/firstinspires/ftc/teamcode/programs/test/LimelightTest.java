package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.switchToAprilTagPipeline;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.switchToArtifactPipeline;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Limelight Test", group = "OpModes")
public class LimelightTest extends CommandOpMode {
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
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        Robot.getInstance().getInstanceLimelight().loop();

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new switchToAprilTagPipeline());
        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new switchToArtifactPipeline());
    }
}