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

        Robot.getInstance().pinpoint.update();
        Pose2D pose = Robot.getInstance().pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.RADIANS);

        double goalX = targetRedGoal ? 144 : 0;
        double goalY = 144;

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        double absoluteAngle = Math.atan2(deltaY, deltaX);
        double relativeAngleRad = AngleUnit.normalizeRadians(absoluteAngle - robotHeading);

        double turretAngle = Math.toDegrees(relativeAngleRad) - 90;
        double finalTurretAngle = AngleUnit.normalizeDegrees(turretAngle);

        double dynamicOffset = 0;

        if (robotX <= 30) {
            double clampedY = Math.max(30, Math.min(robotY, 110));
            double t = (clampedY - 30) / (110 - 30);

            dynamicOffset = -(15 + t * (45 - 15));

            double xFade = Math.max(0, Math.min((35 - robotX) / 10.0, 1.0));
            dynamicOffset *= xFade;
        }

        if (finalTurretAngle > 90) finalTurretAngle -= 360;
        finalTurretAngle = Math.max(-360, Math.min(finalTurretAngle, 90));

        robot.turret.loopAuto(finalTurretAngle - dynamicOffset);

        if (gamepad1.options) robot.pinpoint.resetPosAndIMU();

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading Deg", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Target", finalTurretAngle);
        telemetry.update();
    }
}