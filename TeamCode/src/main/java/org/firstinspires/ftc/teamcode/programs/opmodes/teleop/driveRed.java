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
import static org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR.staticLastAutoY;
import static org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR.targetAngle;

@TeleOp(name = "Drive RED", group = "OpModes")
public class driveRed extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;

    private static final double STICK_EXPONENT   = 3.0;
    private static final double CONSTANT_TERM    = 0.6;
    private static final double LINEAR_COEF      = 0.7;

    public double downY = 0, upY = 0.85, maxDistance = 140, minDistance = 20;
    private double robotX, robotY, distance;
    private ElapsedTime loopTimer = new ElapsedTime();

    private double loopTimeSum = 0;
    private int loopCount = 0;
    private double avgHz = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);  // assuming pipeline 0 works for red too — change if needed

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
                        new WaitCommand(300),
                        new freeServoBarrier(),
                        new ParallelCommandGroup(new startIntakeBack(1), new startIntakeFront(1))
                ))
                .whenInactive(new ParallelCommandGroup(
                        new changeLauncherVelocityState(false),
                        new blockServoBarrier(),
                        new stopIntakeBack(),
                        new stopIntakeFront()
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new changeAimState(!Robot.getInstance().limelightOnlyAim));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new increaseDriverOffset());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new decreaseDriverOffset());
    }

    @Override
    public void run() {
        loopTimer.reset();
        super.run();

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
        robotY = -pose.getY(DistanceUnit.INCH);  // Keep this — makes strafe-right positive

        // Use turret's goal-lock logic (with auto offset if came from auto)
        boolean useLimelight = robot.limelightOnlyAim;

        robot.turret.loop(
                useLimelight ? TurretCR.TurretState.LIMELIGHT_LOCK : TurretCR.TurretState.GOAL_LOCK,
                24,           // targetID — adjust if red uses different AprilTag ID
                true,         // isRedAlliance = true
                robotX,
                robotY,
                Robot.getInstance().driverOffset  // 0 by default, or use your offset system
        );

        distance = robot.turret.getDistance();  // Now correct thanks to turret logic + auto offset

        robot.servoY.setPosition(getServoYPositionFromDistance(distance));

        handleFlywheel();

        updateDriveTelemetry();
    }

    private void handleFlywheel() {
        if (robot.shootFar) {
            robot.flywheel.loopAuto(2300);
        } else {
            robot.flywheel.loop(distance);
        }
    }

    private void updateDriveTelemetry() {
        double currentLoopTime = loopTimer.milliseconds();

        loopTimeSum += currentLoopTime;
        loopCount++;
        if (loopCount >= 10) {
            avgHz = 1000.0 / (loopTimeSum / loopCount);
            loopTimeSum = 0;
            loopCount = 0;
        }

        telemetry.addLine("=== DRIVE RED ACTIVE ===");
        if (robot.pinpoint != null) {
            telemetry.addData("X (Forward)", "%.1f in", robotX);
            telemetry.addData("Y (Strafe)", "%.1f in", robotY);
            telemetry.addData("Distance to Goal", "%.1f in", distance);
            telemetry.addData("Target Angle", "%.1f deg", TurretCR.targetAngle);
            telemetry.addData("staticLastAutoX", "%.1f", staticLastAutoX);
            telemetry.addData("staticLastAutoY", "%.1f", staticLastAutoY);
            telemetry.addData("Driver Offset", "%.1f", Robot.getInstance().driverOffset);
        }

        telemetry.addData("Loop Speed", "%.0f Hz", avgHz);
        telemetry.addData("Loop Time", "%.1f ms", currentLoopTime);
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);
        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);
        return downY + ratio * (upY - downY);
    }
}