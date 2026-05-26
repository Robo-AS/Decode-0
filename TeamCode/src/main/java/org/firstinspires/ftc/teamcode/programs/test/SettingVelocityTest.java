package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetSorterPosition;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.*;
import org.firstinspires.ftc.teamcode.programs.commandbase.mecanum.SetLockMecanumServoState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hood;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name = "Hood and Velocity Test", group = "OpModes")
public class SettingVelocityTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private static final double STICK_EXPONENT = 3.0;
    private static final double CONSTANT_TERM = 0.6;
    private static final double LINEAR_COEF = 0.7;
    public double downY = 0, upY = 0.85, maxDistance = 140, minDistance = 20, robotX, robotY, distance;

    public boolean hue_green, hue_purple;
    private ElapsedTime sensorTimer = new ElapsedTime();
    private boolean sensorsWereActive = false;
    private boolean isReversing = false;
    private boolean sorterMoved = false;
    public boolean isFull = false;

    public double goalX = 0, goalY = 0;
    private double loopTime = 0;
    private int sorterCount = -1;

    public Timer backSensorTimer = new Timer(), lightUpLEDBlue = new Timer();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double targetVelocity = 0, hoodPosition = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        goalX = 133.856315748;
        goalY = 1.6317;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();
        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);
        backSensorTimer.resetTimer();

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        () -> CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new ConditionalCommand(
                                                        new DoesNothingCommand(),
                                                        new ParallelCommandGroup(
                                                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                                                new SetIntakeState(Intake.IntakeState.ON)
                                                        ),
                                                        () -> robot.intake.servoIntakeState == Intake.ServoIntakeState.DOWN && robot.intake.intakeState == Intake.IntakeState.ON
                                                ),
                                                new DoesNothingCommand(),
                                                () -> !isFull
                                        )
                                )
                        )
                );



        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(
                        () -> CommandScheduler.getInstance().schedule(
                                new ParallelCommandGroup(
                                        new SetServoIntakeState(Intake.ServoIntakeState.UP),
                                        new SetIntakeState(Intake.IntakeState.OFF),
                                        new InstantCommand(() -> {
                                            isFull = false;
                                            isReversing = false;
                                            sensorsWereActive = false;
                                        }
                                        )

                                )
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new SetIntakeState(Intake.IntakeState.REVERSED_ON),
                                        new WaitCommand(300),
                                        new SetIntakeState(Intake.IntakeState.OFF)
                                )
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new DoesNothingCommand(),
                                                new ParallelCommandGroup(
                                                        new SetBarrierState(Flywheel.BarrierState.FREE),
                                                        new SetIntakeState(Intake.IntakeState.ON)
                                                ),
                                                () -> robot.flywheel.barrierState == Flywheel.BarrierState.FREE && robot.intake.intakeState == Intake.IntakeState.ON
                                        ),
                                        new ParallelCommandGroup(
                                                new ConditionalCommand(
                                                        new InstantCommand(() -> backSensorTimer.resetTimer()),
                                                        new DoesNothingCommand(),
                                                        () -> (!hue_green && !hue_purple && robot.proximitySensor.getState() && robot.backArtefacts.isPressed())
                                                ),
                                                new ConditionalCommand(
                                                        new DoesNothingCommand(),
                                                        new ParallelCommandGroup(
                                                                new SetBarrierState(Flywheel.BarrierState.FREE),
                                                                new SetIntakeState(Intake.IntakeState.ON)
                                                        ),
                                                        () -> robot.flywheel.barrierState == Flywheel.BarrierState.FREE && robot.intake.intakeState == Intake.IntakeState.ON
                                                ),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(250),
                                                                new InstantCommand(() -> robot.servoSorter.setPosition(0.61)),
                                                                new WaitCommand(150),
                                                                new InstantCommand(() -> sorterMoved = false)
                                                        ),
                                                        new DoesNothingCommand(),
                                                        () -> (!hue_green && !hue_purple && robot.proximitySensor.getState() && robot.backArtefacts.isPressed()) && backSensorTimer.getElapsedTime() >= 10
                                                )),
                                        () -> !sorterMoved
                                )
                        )
                );



        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenInactive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new DoesNothingCommand(),
//                                        new ParallelCommandGroup(
//                                                new SetBarrierState(Flywheel.BarrierState.BLOCK),
//                                                new SetIntakeState(Intake.IntakeState.OFF)
//                                        ),
                                        new SequentialCommandGroup(
                                          new SetIntakeState(Intake.IntakeState.OFF),
                                          new WaitCommand(250),
                                          new SetBarrierState(Flywheel.BarrierState.BLOCK)
                                        ),
                                        () -> robot.flywheel.barrierState == Flywheel.BarrierState.BLOCK && robot.intake.intakeState == Intake.IntakeState.OFF
                                )
                        )
                );


        Trigger farZoneShootingTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8);

        farZoneShootingTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new DoesNothingCommand(),
                                        new SequentialCommandGroup(
                                                new changeLauncherVelocityState(true),
                                                new SetHoodServoState(Hood.HoodServoState.FAR_ZONE),
                                                new WaitCommand(300),
                                                new SetBarrierState(Flywheel.BarrierState.FREE),
                                                new SetIntakeState(Intake.IntakeState.ON)
                                        ),
                                        () -> robot.hood.hoodServoState == Hood.HoodServoState.FAR_ZONE
                                )
                        )
                );

        farZoneShootingTrigger
                .whenInactive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new DoesNothingCommand(),
                                        new ParallelCommandGroup(
                                                new SetHoodServoState(Hood.HoodServoState.AUTOMATED),
                                                new changeLauncherVelocityState(false),
                                                new SetBarrierState(Flywheel.BarrierState.BLOCK),
                                                new SetIntakeState(Intake.IntakeState.OFF)
                                        ),
                                        () -> robot.hood.hoodServoState == Hood.HoodServoState.AUTOMATED
                                )
                        )
                );

        Trigger intakeBack = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8);

        intakeBack
                .whenActive(
                        new ParallelCommandGroup(
                                new ConditionalCommand(
                                        new DoesNothingCommand(),
                                        new ParallelCommandGroup(
                                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                                new SetIntakeState(Intake.IntakeState.ON)
                                        ),
                                        () -> robot.intake.servoIntakeState == Intake.ServoIntakeState.DOWN && robot.intake.intakeState == Intake.IntakeState.ON
                                ),
                                new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new InstantCommand(() -> robot.servoSorter.setPosition(0.24)),
                                            new InstantCommand(() -> sorterMoved = true)
                                    ),
                                    new DoesNothingCommand(),
                                    () ->  (hue_green || hue_purple || !robot.proximitySensor.getState()) && !robot.backArtefacts.isPressed()
                                )
                        ));

        intakeBack
                .whenInactive(
                        () -> CommandScheduler.getInstance().schedule(
                                new SetIntakeState(Intake.IntakeState.OFF)
                        )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new changeAimState(!Robot.getInstance().limelightOnlyAim));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new increaseDriverOffset());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new decreaseDriverOffset());
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(
                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new SetLockMecanumServoState(Mecanum.LockMecanumState.ENGAGED),
                                () -> robot.mecanum.lockMecanumState == Mecanum.LockMecanumState.ENGAGED
                        )
                )
                .whenReleased(
                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new SetLockMecanumServoState(Mecanum.LockMecanumState.DISENGAGED),
                                () -> robot.mecanum.lockMecanumState == Mecanum.LockMecanumState.DISENGAGED
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(
                        () -> Robot.getInstance().turret.applyBlueResetClose_BLUE()
                )
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(
                        () -> Robot.getInstance().turret.applyBlueResetFar_BLUE()
                )
        );
    }




    private void triggerReverseSequence() {
        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetServoIntakeState(Intake.ServoIntakeState.UP),
                        new InstantCommand(() -> {
                            isReversing = true;
                            isFull = true;
                            robot.intake.updateIntakeMotor(Intake.IntakeState.REVERSED_ON);
                        })
                ),
                new WaitCommand(25),
                new InstantCommand(() -> {
                    isReversing = false;
                    robot.intake.updateIntakeMotor(Intake.IntakeState.OFF);
                })
        ));
    }

    private void handleFrontIntakeLogic() {
        boolean zone3 = !robot.proximitySensor.getState() || hue_green || hue_purple;
        boolean currentlyActive = (!robot.are3Artefacts_1.isPressed() || !robot.are3Artefacts_2.isPressed()) &&
                !robot.frontArtefacts.isPressed() && zone3;

        if (currentlyActive || ((!robot.are3Artefacts_2.isPressed() || !robot.are3Artefacts_1.isPressed()) && !robot.frontArtefacts.isPressed())) {
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

        boolean delayPassed = sensorTimer.milliseconds() > 500;

        if (count >= 3) {
            if (delayPassed) robot.led.setPosition(0.611);
        } else if (count == 2 && !robot.frontArtefacts.isPressed() && (!robot.proximitySensor.getState() || hue_green || hue_purple)) {
            robot.led.setPosition(0.388);
        } else if (count == 1) {
            robot.led.setPosition(0.3);
        }

        if((!robot.are3Artefacts_2.isPressed() || !robot.are3Artefacts_1.isPressed()) && !robot.frontArtefacts.isPressed() && count != 3) {
            if (delayPassed) robot.led.setPosition(0.475);
        }

        if(!robot.backArtefacts.isPressed() && (hue_purple || hue_green || !robot.proximitySensor.getState()) && sorterMoved) {
            robot.led.setPosition(0.611);
        }
    }

    @Override
    public void run() {
        super.run();
        robot.loop();

        hue_purple = robot.pin0.getState();
        hue_green = robot.pin1.getState();

        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double x_input = (Math.pow(lx, STICK_EXPONENT) + LINEAR_COEF * lx) * CONSTANT_TERM;
        double y_input = (Math.pow(ly, STICK_EXPONENT) + LINEAR_COEF * ly) * CONSTANT_TERM;
        double rx_final = (Math.pow(rx, STICK_EXPONENT) + LINEAR_COEF * rx) * CONSTANT_TERM;

        if(gamepad1.x){
            robot.mecanum.set(new PoseRR(0, y_input, 0), 0);
        }
        else robot.mecanum.set(new PoseRR(-x_input, y_input, -rx_final), 0);


        if (gamepad1.left_bumper) {
            handleFrontIntakeLogic();
            updateLEDStatus();
        }

        Pose2D pose = robot.pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = -pose.getY(DistanceUnit.INCH);
        distance = Math.hypot(goalX - TurretCR.staticLastAutoX - robotX, goalY - TurretCR.staticLastAutoY - robotY);

        if(distance > 100.0){
//            goalX = 133.856315748;
//            goalY = 2.6317;
            goalX = 141;
            goalY = 5;
        }
        else{
            goalX = 142;
            goalY = 4;
        }

        boolean useLimelight = robot.limelightOnlyAim;
        robot.turret.loop(
                goalX,
                goalY,
                useLimelight ? TurretCR.TurretState.LIMELIGHT_LOCK : TurretCR.TurretState.GOAL_LOCK,
                20,
                false,
                robotX,
                robotY,
                Robot.getInstance().driverOffset
        );


        robot.hoodServo.setPosition(hoodPosition);
        robot.flywheel.loopAuto(targetVelocity);

        updateDriveTelemetry();
    }

    private void updateDriveTelemetry() {
        if (robot.pinpoint != null) {
            telemetry.addData("ROBOT X", robotX);
            telemetry.addData("ROBOT Y", robotY);
            telemetry.addData("Distance", distance);
            telemetry.addData("Green", hue_green);
            telemetry.addData("Purple", hue_purple);
            telemetry.addData("ZONE 3", (hue_green || hue_purple || !robot.proximitySensor.getState()));
            telemetry.addData("ZONE 2", !robot.backArtefacts.isPressed());
            telemetry.addData("Current Angle", TurretCR.currentTurretPosition);
            telemetry.addData("Target Angle", robot.turret.getTargetAngle());
            telemetry.addData("Current Velocity", robot.flywheel.getCurrentVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("kP", TurretCR.kP);
            telemetry.addData("kI", TurretCR.kI);
            telemetry.addData("kD", TurretCR.kD);
            telemetry.addData("kS", TurretCR.kS);
            telemetry.addData("Back Intake Timer", backSensorTimer.getElapsedTime());
            telemetry.addData("Sorter Moved",sorterMoved);
            telemetry.addData("X1", robot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y1", robot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading 1", robot.pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X1", robot.pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y1", robot.pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading 1", robot.pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Offset", robot.turret.resetOffset);
        }


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }

    private double getSorterPosition(int count){
        if(count % 3 == 0) return 0.5;
        if(count % 3 == 1) return 0.88;
        if(count % 3 == 2) return 0.115;

        return 0.5;
    }
}