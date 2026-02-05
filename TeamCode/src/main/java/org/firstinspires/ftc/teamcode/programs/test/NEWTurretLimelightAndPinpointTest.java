package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeFront;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeFront;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
@TeleOp(name = "!!NEW Pinpoint and Limelight Turret Test!!", group = "OpModes")
public class NEWTurretLimelightAndPinpointTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
    public Pose3D botpose;
    public double downY = 1, upY = 0, maxDistance = 0.004, minDistance = 0.2704;

    public double CAMERA_ANGLE = 18;
    public double CAMERA_HEIGHT = 0.4;
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeTurretHardware(hardwareMap);
        robot.initializeTurret();

        robot.limelight.start();
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(0);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new startIntakeFront(1),
                new WaitCommand(1500),
                new stopIntakeFront()
        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
                new startIntakeFront(-1),
                new WaitCommand(300),
                new stopIntakeFront()
        ));

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;
        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        exponentialJoystickCoord_Y = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;

        double turnSpeed =  -exponentialJoystickCoord_X_TURN;
        PoseRR drive = new PoseRR(-exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y, turnSpeed);
        robot.mecanum.set(drive, 0);

        robot.pinpoint.update();

        LLResult result = robot.limelight.getLatestResult();

        robot.turret.loop(20, false);

        telemetry.addData("Target Angle", TurretCR.targetAngle);
        telemetry.update();
    }
}