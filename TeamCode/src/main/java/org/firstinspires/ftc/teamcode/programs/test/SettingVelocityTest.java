package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.setServoLauncherPosition;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
@Config
@TeleOp(name = "Velocity Test", group = "OpModes")
public class SettingVelocityTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
    public Pose3D botpose;
    public double downY = 0.1, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    FtcDashboard dashboard;

    public static double kP = 0.002;
    public static double kI =0;
    public static double kD = 0.00001;
    public static double kS = 0.0435;
    public static double kV = 0.000275;

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

        robot.limelight.start();
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(1);

        feedforward = new SimpleMotorFeedforward(kS, kV);
        pid_Flywheel = new PIDFController(kP, kI, kD, 0);

        flyWheel1 = Robot.getInstance().launcher1;
        flyWheel2 = Robot.getInstance().launcher2;

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new setServoYPosition(0.5));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new startIntake(1),
                new WaitCommand(2000),
                new stopIntake()
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new setServoLauncherPosition(1),
                new WaitCommand(250),
                new setServoLauncherPosition(0)
        ));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        LLResult result = robot.limelight.getLatestResult();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        ta = result.getTa();
        ty = result.getTy();
        tx = result.getTx();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
        distance = Math.sqrt(x_distance*x_distance + y_distance*y_distance);
        targetAngle = tx;

        pos = getServoYPositionFromDistance(y_distance);
        robot.servoY.setPosition(pos);

        currentVelocity = flyWheel1.getVelocity();

        pid_Flywheel.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        double ff = feedforward.calculate(targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);
        double power = pid + ff;

        flyWheel1.setPower(power);
        flyWheel2.setPower(power);

        telemetry.addData("Distance", distance);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance)
    {
        if(distance < maxDistance) return 0.6;
        if(distance > minDistance) return 0.2;

        double ratio = (minDistance - distance) / (minDistance - maxDistance);
        return downY + ratio * (upY - downY);
    }
}

//0.027 3100
//0.0668 1900
//0.09 1700
//0.182 1500
//0.0562 1950
//0.0454 2800
