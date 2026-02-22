package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeBack;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeFront;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeBack;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeFront;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.blockServoBarrier;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.freeServoBarrier;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@Config
@TeleOp(name = "Velocity Test", group = "OpModes")
public class SettingVelocityTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    public double distance;
    public double downY = 0, upY = 1, maxDistance = 140, minDistance = 20;

    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;
    FtcDashboard dashboard;

    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0.05;
    public static double kV = 0.0003;

    public boolean isRedAlliance = false;

    private DcMotorEx flyWheel1, flyWheel2;

    PIDFController pid_Flywheel;
    SimpleMotorFeedforward feedforward;

    public double CAMERA_ANGLE = 18;
    public double CAMERA_HEIGHT = 0.4;

    public static double targetVelocity = 0, currentVelocity = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        feedforward = new SimpleMotorFeedforward(kS, kV);
        pid_Flywheel = new PIDFController(kP, kI, kD, 0);

        flyWheel1 = Robot.getInstance().launcher1;
        flyWheel2 = Robot.getInstance().launcher2;

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new startIntakeFront(1),
                new startIntakeBack(1),
                new WaitCommand(1500),
                new stopIntakeFront(),
                new stopIntakeBack()
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new freeServoBarrier(),
                new WaitCommand(100),
                new startIntakeFront(1),
                new startIntakeBack(1),
                new WaitCommand(1500),
                new stopIntakeFront(),
                new stopIntakeBack(),
                new blockServoBarrier()
        ));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;
        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        exponentialJoystickCoord_Y = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;

        double turnSpeed = -exponentialJoystickCoord_X_TURN;
        PoseRR drive = new PoseRR(-exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y, turnSpeed);
        robot.mecanum.set(drive, 0);

        robot.pinpoint.update();

        currentVelocity = flyWheel1.getVelocity();

        pid_Flywheel.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        double ff = feedforward.calculate(targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);
        double power = pid + ff;

        flyWheel1.setPower(power);
        flyWheel2.setPower(power);

        double goalY = isRedAlliance ? 152.0 : -8.75;
        double goalX = 136.0;

        Pose2D pose = robot.pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = -pose.getY(DistanceUnit.INCH);

        distance = Math.hypot(goalY - robotY, goalX - robotX);

        robot.servoY.setPosition(getServoYPositionFromDistance(distance));

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", currentVelocity);
        packet.put("Error", targetVelocity - currentVelocity);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Distance", distance);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);

        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);

        return downY + ratio * (upY - downY);
    }
}

/*
55.8133 1600
69.4260 1800
90.9241 1850
102.6444 1900
118.0427 2100
76.2866 1800
130 2300
141.8654 2350
 */