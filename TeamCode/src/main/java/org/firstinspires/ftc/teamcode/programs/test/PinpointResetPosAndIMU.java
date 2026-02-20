package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.changeAimState;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Pinpoint Reset Pos and IMU Test", group = "Test")
public class PinpointResetPosAndIMU extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RunCommand(
                        () -> Robot.getInstance().pinpoint.resetPosAndIMU()
                ));

    }

    @Override
    public void run() {
        robot.pinpoint.update();

        Pose2D pose = robot.pinpoint.getPosition();


        telemetry.addData("X",pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Yaw Scalar", robot.pinpoint.getYawScalar());
        telemetry.addData("Raw X ticks to INCH", robot.pinpoint.getEncoderX() / 505.317);
        telemetry.addData("Raw Y ticks to INCH", robot.pinpoint.getEncoderY() / 505.317);
        telemetry.update();
    }
}