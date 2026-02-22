package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

import static org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR.staticLastAutoX;
import static org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR.targetAngle;

@TeleOp(name = "Drive BLUE", group = "OpModes")
public class driveBlue extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;

    private static final double STICK_EXPONENT   = 3.0;
    private static final double CONSTANT_TERM    = 0.6;
    private static final double LINEAR_COEF      = 0.7;

    private static final double CAMERA_ANGLE     = 18.0;
    private static final double CAMERA_HEIGHT    = 0.4;
    public double downY = 0, upY = 1, maxDistance = 140, minDistance = 20, robotX, robotY, distance;

    private long lastLLUpdate = 0;
    private double currentDistance = 0;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);

        setupControllerBindings();
    }

    private void setupControllerBindings() {
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new SequentialCommandGroup(
                        new liftServoIntake(),
                        new ParallelCommandGroup(new startIntakeBack(1), new startIntakeFront(1))
                ))
                .whenReleased(new SequentialCommandGroup(
                        new lowerServoIntake(),
                        new ParallelCommandGroup(new stopIntakeBack(), new stopIntakeFront())
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new ParallelCommandGroup(new startIntakeFront(-1), new startIntakeBack(-1)),
                        new WaitCommand(300),
                        new ParallelCommandGroup(new stopIntakeFront(), new stopIntakeBack())
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ParallelCommandGroup(
                        new freeServoBarrier(),
                        new startIntakeBack(1),
                        new startIntakeFront(1)
                ))
                .whenReleased(new ParallelCommandGroup(
                        new blockServoBarrier(),
                        new stopIntakeBack(),
                        new stopIntakeFront()
                ));

        new com.arcrobotics.ftclib.command.button.Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(new SequentialCommandGroup(
                        new changeLauncherVelocityState(true),
                        new setServoYPosition(1),
                        new WaitCommand(1200),
                        new freeServoBarrier(),
                        new ParallelCommandGroup(new startIntakeBack(1), new startIntakeFront(1))
                ))
                .whenInactive(new ParallelCommandGroup(
                        new changeLauncherVelocityState(false),
                        new blockServoBarrier(),
                        new stopIntakeBack(),
                        new stopIntakeFront()
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new changeAimState(!Robot.getInstance().limelightOnlyAim));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new increaseDriverOffset());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new decreaseDriverOffset());
    }

    @Override
    public void run() {
        super.run();
        loopTimer.reset();

        robot.update();

        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double x_input = (Math.pow(lx, STICK_EXPONENT) + LINEAR_COEF * lx) * CONSTANT_TERM;
        double y_input = (Math.pow(ly, STICK_EXPONENT) + LINEAR_COEF * ly) * CONSTANT_TERM;
        double rx_final = (Math.pow(rx, STICK_EXPONENT) + LINEAR_COEF * rx) * CONSTANT_TERM;

        robot.mecanum.set(new PoseRR(-x_input, y_input, -rx_final), 0);

        Pose2D pose = robot.pinpoint.getPosition();

        robotX = pose.getX(DistanceUnit.INCH);
        robotY = -pose.getY(DistanceUnit.INCH);

        double goalY = 0;
        double goalX = 144.0;

        boolean useLimelight = robot.limelightOnlyAim;
        robot.turret.loop(
                useLimelight ? TurretCR.TurretState.LIMELIGHT_LOCK : TurretCR.TurretState.GOAL_LOCK,
                20,
                false,
                robotX,
                robotY,
                Robot.getInstance().driverOffset
        );

        distance = robot.turret.getDistance();
        if(Robot.getInstance().wasUpperBlueAutoRan) distance -= 70.0;

        robot.servoY.setPosition(getServoYPositionFromDistance(distance));

        handleFlywheel();
        updateDriveTelemetry();
    }

    private void handleFlywheel() {
        if (robot.shootFar) {
            robot.flywheel.loopAuto(2300);
        } else{
            robot.flywheel.loop(distance);
        }
    }

    private void updateDriveTelemetry() {
        if (robot.pinpoint != null) {
            telemetry.addData("ROBOT X", robotX);
            telemetry.addData("ROBOT Y", robotY);
            telemetry.addData("Distance", distance);
            telemetry.addData("Driver Offset", Robot.getInstance().driverOffset);
        }

        double loopTimeMs = loopTimer.milliseconds();
        double hz = (loopTimeMs > 0) ? (1000.0 / loopTimeMs) : 0;

        telemetry.addData("Loop Time", "%.1f ms", hz);
        telemetry.update();
    }

    private boolean isLimelightOffline() {
        LLResult res = robot.limelight.getLatestResult();
        return res == null || !res.isValid();
    }

    public double getServoYPositionFromDistance(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);

        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);

        return downY + ratio * (upY - downY);
    }
}