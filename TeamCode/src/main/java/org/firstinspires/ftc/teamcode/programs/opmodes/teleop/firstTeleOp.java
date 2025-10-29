package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

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

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.Shoot;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.setServoLauncherPosition;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.startLauncher;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Drive", group = "OpModes")
public class firstTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap, (MultipleTelemetry) telemetry);
        robot.initialize();

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new startIntake(1),
                new WaitCommand(2000),
                new stopIntake()
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
                new startIntake(-1),
                new WaitCommand(1000),
                new stopIntake()
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new setServoLauncherPosition(1),
                new WaitCommand(250),
                new setServoLauncherPosition(0)
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new setServoYPosition(0.3));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new setServoYPosition(0.5));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.launcher.setPower(1);

        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;
        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        exponentialJoystickCoord_Y = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;

        double turnSpeed =  -exponentialJoystickCoord_X_TURN;
        PoseRR drive = new PoseRR(-exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y, turnSpeed);
        robot.mecanum.set(drive, 0);

        robot.pinpoint.update();
       // robot.llwrapped.loop();
        robot.turret.loop(20);
    }
}