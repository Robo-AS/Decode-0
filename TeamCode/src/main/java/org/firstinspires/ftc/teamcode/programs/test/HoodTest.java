package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;

@TeleOp(name = "Hood Test", group = "OpModes")
public class HoodTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
    public Pose3D botpose;
    public double downY = 0, upY = 1, maxDistance = 0.004, minDistance = 0.2704;
    private double lastServoY = downY;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;
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
        robot.limelight.pipelineSwitch(0);
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new setServoYPosition(0.5));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double turnInput = -((Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm);
        double forwardInput = -(Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        double strafeInput = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;
        PoseRR drive = new PoseRR(forwardInput, strafeInput, turnInput);
        robot.mecanum.set(drive, 0);

        robot.launcher1.setVelocity(2600);
        robot.launcher2.setVelocity(2600);

        LLResult result = robot.limelight.getLatestResult();

        if(result != null && result.isValid()) {
            botpose = result.getBotpose_MT2();
            ta = result.getTa();
            tx = result.getTx();
            ty = result.getTy();

            y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
            x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
            distance = y_distance;
            targetAngle = tx;

            pos = getServoYPositionFromDistance(distance);
            robot.servoY.setPosition(pos);

            telemetry.addData("Target Area", ta);
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Distance", distance);
            telemetry.addData("ServoY position", pos);
        }

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
