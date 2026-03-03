package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;
import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Drive BLUE", group = "OpModes")
public class driveBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private static final double STICK_EXPONENT = 3.0;
    private static final double CONSTANT_TERM = 0.6;
    private static final double LINEAR_COEF = 0.7;
    public double downY = 0, upY = 0.85, maxDistance = 140, minDistance = 20, robotX, robotY, distance, loopTime;
    public boolean hue_green, hue_purple;
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime sensorTimer = new ElapsedTime();
    private boolean sensorsWereActive = false;
    private boolean isReversing = false;
    private boolean isFull = false;

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
                .whileHeld(new RunCommand(() -> {
                    if (!isFull) {
                        new SetServoIntakeState(Intake.ServoIntakeState.LOWER).schedule();
                        new SetIntakeState(Intake.IntakeState.ON).schedule();
                    }
                }))
                .whenReleased(new SequentialCommandGroup(
                        new SetServoIntakeState(Intake.ServoIntakeState.LIFT),
                        new ParallelCommandGroup(
                                new SetIntakeState(Intake.IntakeState.OFF),
                                new InstantCommand(() -> {
                                    isFull = false;
                                    isReversing = false;
                                    sensorsWereActive = false;
                                })
                        )
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new SetIntakeState(Intake.IntakeState.REVERSED_ON),
                        new WaitCommand(300),
                        new SetIntakeState(Intake.IntakeState.OFF)
                ));

        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whileActiveContinuous(new ParallelCommandGroup(
                        new SetBarrierState(Flywheel.BarrierState.FREE),
                        new SetIntakeState(Intake.IntakeState.ON)
                ))
                .whenInactive(new ParallelCommandGroup(
                        new SetBarrierState(Flywheel.BarrierState.BLOCK),
                        new SetIntakeState(Intake.IntakeState.OFF)
                ));

        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(new SequentialCommandGroup(
                        new changeLauncherVelocityState(true),
                        new setServoYPosition(1),
                        new WaitCommand(300),
                        new SetBarrierState(Flywheel.BarrierState.FREE),
                        new SetIntakeState(Intake.IntakeState.ON)
                ))
                .whenInactive(new ParallelCommandGroup(
                        new changeLauncherVelocityState(false),
                        new SetBarrierState(Flywheel.BarrierState.BLOCK),
                        new SetIntakeState(Intake.IntakeState.OFF)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new changeAimState(!Robot.getInstance().limelightOnlyAim));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new increaseDriverOffset());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new decreaseDriverOffset());
    }

    private void triggerReverseSequence() {
        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetServoIntakeState(Intake.ServoIntakeState.LIFT),
                        new InstantCommand(() -> {
                            isReversing = true;
                            isFull = true;
                            robot.intake.updateIntake(Intake.IntakeState.REVERSED_ON);
                        })
                ),
                new WaitCommand(50),
                new InstantCommand(() -> {
                    isReversing = false;
                    robot.intake.updateIntake(Intake.IntakeState.OFF);
                })
        ));
    }

    private void handleIntakeLogic() {
        boolean zone3 = !robot.proximitySensor.getState() || hue_green || hue_purple;
        boolean currentlyActive = (!robot.are3Artefacts_1.isPressed() || !robot.are3Artefacts_2.isPressed()) &&
                !robot.frontArtefacts.isPressed() && zone3;

        if (currentlyActive) {
            if (!sensorsWereActive) {
                sensorTimer.reset();
                sensorsWereActive = true;
            }
            if (sensorTimer.milliseconds() > 500 && !isFull) {
                triggerReverseSequence();
            }
        } else {
            sensorsWereActive = false;
        }
    }

    private void updateLEDStatus() {
        int count = 0;
        if (!robot.are3Artefacts_1.isPressed() || !robot.are3Artefacts_2.isPressed()) count++;
        if (!robot.frontArtefacts.isPressed()) count++;
        if (!robot.proximitySensor.getState() || hue_green || hue_purple) count++;

        if (count >= 3) {
            robot.led.setPosition(0.475);
        } else if (count == 2) {
            robot.led.setPosition(0.388);
        } else if (count == 1) {
            robot.led.setPosition(0.333);
        } else {
            robot.led.setPosition(0.277);
        }
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        super.run();
        loopTimer.reset();
        robot.update();

        hue_purple = robot.pin0.getState();
        hue_green = robot.pin1.getState();

        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double x_input = (Math.pow(lx, STICK_EXPONENT) + LINEAR_COEF * lx) * CONSTANT_TERM;
        double y_input = (Math.pow(ly, STICK_EXPONENT) + LINEAR_COEF * ly) * CONSTANT_TERM;
        double rx_final = (Math.pow(rx, STICK_EXPONENT) + LINEAR_COEF * rx) * CONSTANT_TERM;

        robot.mecanum.set(new PoseRR(-x_input, y_input, -rx_final), 0);

        robot.intake.loop();

        if (gamepad1.left_bumper) {
            handleIntakeLogic();
        }

        updateLEDStatus();

        Pose2D pose = robot.pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = -pose.getY(DistanceUnit.INCH);

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
        if (robot.pinpoint != null) {
            telemetry.addData("ROBOT X", robotX);
            telemetry.addData("ROBOT Y", robotY);
            telemetry.addData("Distance", distance);
            telemetry.addData("Lidar State", robot.proximitySensor.getState());
            telemetry.addData("Hue Green", hue_green);
            telemetry.addData("Hue Purple", hue_purple);
        }
        double loopTimeMs = loopTimer.milliseconds();
        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }

    public double getServoYPositionFromDistance(double distance) {
        double clippedDistance = Range.clip(distance, minDistance, maxDistance);
        double ratio = (clippedDistance - minDistance) / (maxDistance - minDistance);
        return downY + ratio * (upY - downY);
    }
}