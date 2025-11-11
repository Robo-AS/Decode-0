package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
@Config
@TeleOp(name = "Velocity Test", group = "OpModes")
public class SettingVelocityTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
    public Pose3D botpose;
    public double downY = 0.2, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;

    public static double velocity = 0.0;

    public double CAMERA_ANGLE = 18;
    public double CAMERA_HEIGHT = 0.4;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();
        robot.limelight.start();
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(1);
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new setServoYPosition(0.5));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.launcher1.setVelocity(velocity);
        robot.launcher2.setVelocity(velocity);

        LLResult result = robot.limelight.getLatestResult();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        ta = result.getTa();
        ty = result.getTy();
        tx = result.getTx();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
        distance = y_distance;
        targetAngle = tx;

        pos = getServoYPositionFromDistance(distance);
        robot.servoY.setPosition(pos);

        telemetry.addData("Distance", distance);
        telemetry.addData("Velocity", velocity);
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
