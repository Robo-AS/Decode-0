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
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.setServoLauncherPosition;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@Config
@TeleOp(name = "Velocity Test", group = "OpModes")
public class SettingVelocityTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private FtcDashboard dashboard;

    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;

    public static double downY = 0.05, upY = 0.95;
    public double maxDistance = 0.004, minDistance = 0.2704;

    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0.1;
    public static double kV = 0.000167;

    private DcMotorEx flyWheel1, flyWheel2;

    private PIDFController pid_Flywheel;
    private SimpleMotorFeedforward feedforward;

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

        pid_Flywheel = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        flyWheel1 = robot.launcher1;
        flyWheel2 = robot.launcher2;

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new startIntake(1),
                        new WaitCommand(2000),
                        new stopIntake()
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new setServoLauncherPosition(1),
                        new WaitCommand(250),
                        new setServoLauncherPosition(0)
                ));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        exponentialJoystickCoord_X_TURN =
                (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;
        exponentialJoystickCoord_X_FORWARD =
                (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        exponentialJoystickCoord_Y =
                (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;

        double turnSpeed = -exponentialJoystickCoord_X_TURN;
        PoseRR drive = new PoseRR(
                -exponentialJoystickCoord_X_FORWARD,
                exponentialJoystickCoord_Y,
                turnSpeed
        );
        robot.mecanum.set(drive, 0);

        LLResult result = robot.limelight.getLatestResult();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        if (result != null && result.isValid()) {

            ta = result.getTa();
            ty = result.getTy();
            tx = result.getTx();

            y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
            x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT)
                    * Math.tan(Math.toRadians(tx));
            distance = Math.sqrt(x_distance * x_distance + y_distance * y_distance);
            targetAngle = tx;

            boolean seesTargetID = false;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == 20) {
                    seesTargetID = true;
                    break;
                }
            }

            if (seesTargetID) {
                pos = getServoYPositionFromDistance(y_distance);
                robot.servoY.setPosition(pos);
            }
        }

        currentVelocity = flyWheel2.getVelocity();

        double ff = feedforward.calculate(targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);
        double power = pid + ff;

        if (power < 0) power = 0;

        flyWheel1.setPower(power);
        flyWheel2.setPower(power);

        telemetry.addData("distance", distance);
        telemetry.addData("servoY", robot.servoY.getPosition());
        telemetry.addData("velocity", currentVelocity);
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance) {
        if (distance < maxDistance) return upY;
        if (distance > minDistance) return downY;

        double ratio = (minDistance - distance) / (minDistance - maxDistance);
        return downY + ratio * (upY - downY);
    }
}

//0.1468 1800
//0.1113 1800
//0.0735 2000
//0.062  2000
//0.0445 2300
//0.0587 2300