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

@TeleOp(name = "Pinpoint Manual Test", group = "Test")
public class PinpointManualTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.configurePinpoint();
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