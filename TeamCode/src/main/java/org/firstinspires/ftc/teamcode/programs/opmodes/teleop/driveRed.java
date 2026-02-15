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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Drive RED", group = "OpModes")
public class driveRed extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;

    private static final double STICK_EXPONENT = 3.0;
    private static final double CONSTANT_TERM = 0.6;
    private static final double LINEAR_COEF = 0.7;
    private static final double CAMERA_ANGLE = 18.0;
    private static final double CAMERA_HEIGHT = 0.4;
    private static final double MIN_DIST = 0.2704;
    private static final double MAX_DIST = 0.004;

    private double currentDistance = 0;
    private final int RED_GOAL_ID = 24;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);

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
                        new WaitCommand(800),
                        new freeServoBarrier(),
                        new ParallelCommandGroup(new startIntakeBack(1), new startIntakeFront(1))
                ))
                .whenInactive(new ParallelCommandGroup(
                        new changeLauncherVelocityState(false),
                        new blockServoBarrier(),
                        new stopIntakeBack(),
                        new stopIntakeFront()
                ));
    }

    @Override
    public void run() {
        super.run();

        robot.pinpoint.update();

        double driveX = (Math.pow(gamepad1.left_stick_x, STICK_EXPONENT) + LINEAR_COEF * gamepad1.left_stick_x) * CONSTANT_TERM;
        double driveY = (Math.pow(gamepad1.left_stick_y, STICK_EXPONENT) + LINEAR_COEF * gamepad1.left_stick_y) * CONSTANT_TERM;
        double driveRot = (Math.pow(gamepad1.right_stick_x, STICK_EXPONENT) + LINEAR_COEF * gamepad1.right_stick_x) * CONSTANT_TERM;

        robot.mecanum.set(new PoseRR(-driveX, driveY, -driveRot), 0);

        LLResult result = robot.limelight.getLatestResult();
        boolean seesGoal = false;

        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == RED_GOAL_ID) {
                        seesGoal = true;
                        processVision(result);
                        break;
                    }
                }
            }
        }

        if(Robot.getInstance().limelightOnlyAim)
            robot.turret.loop(TurretCR.TurretState.LIMELIGHT_LOCK, 24, false);
        else
            robot.turret.loop(TurretCR.TurretState.GOAL_LOCK, 24, false);

        if (robot.shootFar) {
            robot.flywheel.loopAuto(2400);
        } else if (!seesGoal) {
            robot.flywheel.loopAuto(1900);
        }

        telemetry.addData("Turret Angle", TurretCR.targetAngle);
        telemetry.addData("Distance", currentDistance);
        telemetry.addData("Flywheel Actual", robot.flywheel.getCurrentVelocity());
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
                if (fr.getFiducialId() == 24) { targetFound = true; break; }
            }
        }

        if (targetFound) {
            robot.flywheel.loop(currentDistance);
            double sPos = (currentDistance < MAX_DIST) ? 1.0 : (currentDistance > MIN_DIST) ? 0.0 : (MIN_DIST - currentDistance) / (MIN_DIST - MAX_DIST);
            robot.servoY.setPosition(sPos);
        }
    }
}