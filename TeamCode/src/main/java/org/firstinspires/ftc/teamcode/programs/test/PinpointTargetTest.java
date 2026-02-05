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

@TeleOp(name = "Pinpoint Turret Logic Test", group = "Test")
public class PinpointTargetTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;
    public static boolean targetRedGoal = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.initializeHardware(hardwareMap);
        robot.initialize();
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

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.RADIANS);

        double goalX = targetRedGoal ? 144 : 0;
        double goalY = 144;

        double angleToGoalField = Math.atan2(goalY - robotY, goalX - robotX);
        double relativeAngleRad = angleToGoalField - robotHeading;

        double turretAngle = Math.toDegrees(AngleUnit.normalizeRadians(relativeAngleRad)) - 90;

        if(turretAngle > 180) turretAngle = 180;
        else if(turretAngle < -180) turretAngle = -180;

        robot.turret.loopAuto(turretAngle);

        if (gamepad1.options) robot.pinpoint.resetPosAndIMU();

        telemetry.addData("Target Goal", targetRedGoal ? "RED" : "BLUE");
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Target Angle", turretAngle);
        telemetry.update();
    }
}