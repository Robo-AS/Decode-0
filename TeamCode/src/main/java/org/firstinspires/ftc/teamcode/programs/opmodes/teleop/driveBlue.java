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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

import static org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR.targetAngle;

@TeleOp(name = "Drive BLUE - Corrected", group = "OpModes")
public class driveBlue extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;

    private static final double STICK_EXPONENT   = 3.0;
    private static final double CONSTANT_TERM    = 0.6;
    private static final double LINEAR_COEF      = 0.7;

    private static final double CAMERA_ANGLE     = 18.0;
    private static final double CAMERA_HEIGHT    = 0.4;
    private static final double MIN_DIST         = 0.2704;
    private static final double MAX_DIST         = 0.004;

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

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new changeAimState());
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

        if (System.currentTimeMillis() - lastLLUpdate > 25) {
            LLResult result = robot.limelight.getLatestResult();
            lastLLUpdate = System.currentTimeMillis();
            if (result != null && result.isValid()) {
                processVision(result);
            }
        }

        boolean useLimelight = robot.limelightOnlyAim;
        robot.turret.loop(
                useLimelight ? TurretCR.TurretState.LIMELIGHT_LOCK : TurretCR.TurretState.GOAL_LOCK,
                20,
                false
        );

        handleFlywheel();
        updateDriveTelemetry();
    }

    private void handleFlywheel() {
        if (robot.shootFar) {
            robot.flywheel.loopAuto(2400);
        } else if (isLimelightOffline()) {
            robot.flywheel.loopAuto(1900);
        }
    }

    private void updateDriveTelemetry() {
        telemetry.addLine("=== Odometry (Global) ===");
        if (robot.pinpoint != null) {
            Pose2D pos = robot.pinpoint.getPosition();
            telemetry.addData("X (Forward)", "%.1f in", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y (Strafe)", "%.1f in", pos.getY(DistanceUnit.INCH));

            double rawH = pos.getHeading(AngleUnit.RADIANS);
            double corrH = AngleUnit.normalizeRadians(-rawH + Math.PI / 2);
            telemetry.addData("Heading (Raw)", "%.1f deg", Math.toDegrees(rawH));
            telemetry.addData("Heading (Field)", "%.1f deg", Math.toDegrees(corrH));
        }

        telemetry.addLine("=== Turret & Vision ===");
        telemetry.addData("Target Angle", "%.2f deg", targetAngle);
        telemetry.addData("Mode", robot.turret.currentState);
        telemetry.addData("LL Distance", "%.3f", currentDistance);

        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());
        telemetry.update();
    }

    private void processVision(LLResult result) {
        double tx = result.getTx();
        double ty = result.getTy();

        double distY = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        double distX = Math.sqrt(distY * distY + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
        currentDistance = Math.sqrt(distX * distX + distY * distY);

        boolean targetFound = false;
        if (result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20) {
                    targetFound = true;
                    break;
                }
            }
        }

        if (targetFound) {
            robot.flywheel.loop(currentDistance);
            double sPos = (currentDistance < MAX_DIST) ? 1.0 :
                    (currentDistance > MIN_DIST) ? 0.0 :
                            (MIN_DIST - currentDistance) / (MIN_DIST - MAX_DIST);
            robot.servoY.setPosition(sPos);
        }
    }

    private boolean isLimelightOffline() {
        LLResult res = robot.limelight.getLatestResult();
        return res == null || !res.isValid();
    }
}