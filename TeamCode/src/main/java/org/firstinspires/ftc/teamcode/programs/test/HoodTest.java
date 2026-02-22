package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;

@TeleOp(name = "Hood Test", group = "OpModes")
public class HoodTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
    public Pose3D botpose;
    public double downY = 0, upY = 1, maxDistance = 140, minDistance = 20;
    private double lastServoY = downY;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;
    public double CAMERA_ANGLE = 18;
    public double CAMERA_HEIGHT = 0.4;
    private static final double STICK_EXPONENT   = 3.0;
    private static final double CONSTANT_TERM    = 0.6;
    private static final double LINEAR_COEF      = 0.7;

    private boolean isRedAlliance = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();
        robot.limelight.start();
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(0);
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new setServoYPosition(0.5));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double x_input = (Math.pow(lx, STICK_EXPONENT) + LINEAR_COEF * lx) * CONSTANT_TERM;
        double y_input = (Math.pow(ly, STICK_EXPONENT) + LINEAR_COEF * ly) * CONSTANT_TERM;
        double rx_final = (Math.pow(rx, STICK_EXPONENT) + LINEAR_COEF * rx) * CONSTANT_TERM;

        robot.mecanum.set(new PoseRR(-x_input, y_input, -rx_final), 0);
        robot.mecanum.set(new PoseRR(-x_input, y_input, -rx_final), 0);

        robot.flywheel.loopAuto(1900);
        robot.pinpoint.update();

        LLResult result = robot.limelight.getLatestResult();

        double goalY = isRedAlliance ? 152.0 : -8.75;
        double goalX = 136.0;

        Pose2D pose = robot.pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = -pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.RADIANS);

        double distance = Math.hypot(goalX - robotX, goalY - robotY);

        double pos = getServoYPositionFromDistance(distance);
        robot.servoY.setPosition(pos);

        telemetry.addData("Distance", distance);
        telemetry.addData("Servo Position", pos);
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);

        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);

        return downY + ratio * (upY - downY);
    }
}
