package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Pinpoint Localization Test", group = "Test")
public class PinpointLocalizationTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.initializeHardware(hardwareMap);
        robot.initializeTest();
    }

    @Override
    public void run() {
        double x_input = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        double y_input = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;
        double turn_input = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;

        PoseRR drive = new PoseRR(-x_input, y_input, -turn_input);
        robot.mecanum.set(drive, 0);

        robot.pinpoint.update();

        Pose2D pose = robot.pinpoint.getPosition();

        telemetry.addData("X",pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Raw X ticks", robot.pinpoint.getEncoderX());
        telemetry.addData("Raw Y ticks", robot.pinpoint.getEncoderY());
        telemetry.update();
    }
}