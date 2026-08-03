package org.firstinspires.ftc.teamcode.programs.opmodes.auto.commandbase;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.SetIntakeState;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.SetServoIntakeState;
import org.firstinspires.ftc.teamcode.programs.commandbase.launcher.SetBarrierState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "🏰IST far blue 🔵")
public class ISTbottomBlueAuto extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double loopTime = 0;
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private PathChain spike3, outtake1, cycleLOW1, outtake2, cycleHIGH1, outtake3, cycleLOW2, outtake4, cycleHIGH2, outtake5, leave;

    private void buildPaths() {
        spike3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(56.000, 8.000), new Pose(53, 31.653), new Pose(13.340, 36.340))
        ).setTangentHeadingInterpolation().setBrakingStrength(3.5).build();

        outtake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(13.340, 36.340), new Pose(53.000, 12.000))
        ).setTangentHeadingInterpolation().setBrakingStrength(3.5).setReversed().build();

        cycleLOW1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 12.000), new Pose(15, 10.566)))
                .setTangentHeadingInterpolation().setBrakingStrength(3.5)
                .addPath(new BezierLine(new Pose(15, 10.566), new Pose(7.5, 8.500)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();

        outtake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(7.5, 8.500), new Pose(53.000, 12.000))
        ).setConstantHeadingInterpolation(Math.toRadians(184.3)).setBrakingStrength(3.5).build();

        cycleHIGH1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 12.000), new Pose(26.000, 25.500)))
                .setTangentHeadingInterpolation().setBrakingStrength(3.5)
                .addPath(new BezierLine(new Pose(26.000, 25.500), new Pose(10, 25.500)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();

        outtake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(10, 25.500), new Pose(53.000, 12.000))
        ).setConstantHeadingInterpolation(Math.toRadians(163.9)).setBrakingStrength(3.5).build();

        cycleHIGH2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 12.000), new Pose(26, 25.500)))
                .setTangentHeadingInterpolation().setBrakingStrength(3.5)
                .addPath(new BezierLine(new Pose(26, 25.500), new Pose(10, 25.500)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();

        outtake5 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(10, 25.500), new Pose(53.000, 12.000))
        ).setConstantHeadingInterpolation(Math.toRadians(163.9)).setBrakingStrength(3.5).build();

        cycleLOW2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(56.000, 8.000), new Pose(53, 31.653), new Pose(13.340, 36.340))
        ).setTangentHeadingInterpolation().setBrakingStrength(3.5).build();

        outtake4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(13.340, 36.340), new Pose(53.000, 12.000))
        ).setTangentHeadingInterpolation().setBrakingStrength(3.5).setReversed().build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(new Pose(56.000, 12.000), new Pose(48.000, 17.000))
        ).setConstantHeadingInterpolation(90).setBrakingStrength(3.5).build();
    }

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.initializeHardwareAuto(hardwareMap);
        robot.initializeAuto();

        robot.intake.intakeState = Intake.IntakeState.OFF;
        robot.intake.servoIntakeState = Intake.ServoIntakeState.DOWN;
        robot.flywheel.barrierState = Flywheel.BarrierState.BLOCK;
    }

    private boolean isRobotFull() {
        int count = 0;
        if (!robot.are3Artefacts_1.isPressed() || !robot.are3Artefacts_2.isPressed()) count++;
        if (!robot.frontArtefacts.isPressed()) count++;
        if (!robot.proximitySensor.getState() || robot.pin1.getState() || robot.pin0.getState()) count++;
        return count >= 3;
    }

    private Command followPath(PathChain path, boolean holdEnd) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.followPath(path, holdEnd)),
                new WaitUntilCommand(() -> !follower.isBusy())
        );
    }

    private Command followPathWithEarlyIntake(PathChain path, boolean holdEnd) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.followPath(path, holdEnd)),
                new SetIntakeState(Intake.IntakeState.ON),
                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.3 || !follower.isBusy()),
                new SetIntakeState(Intake.IntakeState.OFF),
                new WaitUntilCommand(() -> !follower.isBusy())
        );
    }

    private Command followPathWithMidpointIntake(PathChain path) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(this::isRobotFull).withTimeout(2500),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.followPath(path, false)),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.3),
                                new SetIntakeState(Intake.IntakeState.ON),
                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.98 || !follower.isBusy())
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                                new SetIntakeState(Intake.IntakeState.REVERSED_ON),
                                new WaitCommand(25),
                                new SetIntakeState(Intake.IntakeState.OFF)
                        ),
                        new SetIntakeState(Intake.IntakeState.OFF),
                        this::isRobotFull
                )
        );
    }

    private Command followChainedCyclePath(PathChain path) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(this::isRobotFull).withTimeout(2000),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.followPath(path, true)),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.15),
                                new SetIntakeState(Intake.IntakeState.ON),
                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.48),
                                new SetServoIntakeState(Intake.ServoIntakeState.AUTO_GATE),
                                new WaitUntilCommand(() -> !follower.isBusy())
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                                new SetIntakeState(Intake.IntakeState.REVERSED_ON),
                                new WaitCommand(25),
                                new SetIntakeState(Intake.IntakeState.OFF)
                        ),
                        new SetIntakeState(Intake.IntakeState.OFF),
                        this::isRobotFull
                ),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN)
        );
    }

    private Command shootingSequence() {
        return new SequentialCommandGroup(
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SetIntakeState(Intake.IntakeState.ON),
                                new DoesNothingCommand(),
                                () -> robot.intake.intakeState == Intake.IntakeState.OFF
                        ),
                        new ConditionalCommand(
                                new SetBarrierState(Flywheel.BarrierState.FREE),
                                new DoesNothingCommand(),
                                () -> robot.flywheel.barrierState == Flywheel.BarrierState.BLOCK
                        )
                ),
                new WaitCommand(1200),
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SetBarrierState(Flywheel.BarrierState.BLOCK),
                                new DoesNothingCommand(),
                                () -> robot.flywheel.barrierState == Flywheel.BarrierState.FREE
                        ),
                        new ConditionalCommand(
                                new SetIntakeState(Intake.IntakeState.OFF),
                                new DoesNothingCommand(),
                                () -> robot.intake.intakeState != Intake.IntakeState.OFF
                        )
                )
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Command auto = new SequentialCommandGroup(
                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                new WaitCommand(1200),
                shootingSequence(),

                followPathWithMidpointIntake(spike3),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                new WaitCommand(5),

                followPathWithEarlyIntake(outtake1, false),
                shootingSequence(),

                followChainedCyclePath(cycleLOW1),

                followPathWithEarlyIntake(outtake2, false),
                shootingSequence(),

                followChainedCyclePath(cycleHIGH1),

                followPathWithEarlyIntake(outtake3, false),
                shootingSequence(),

                followChainedCyclePath(cycleLOW2),

                followPathWithEarlyIntake(outtake4, false),
                shootingSequence(),

                followChainedCyclePath(cycleHIGH2),

                followPathWithEarlyIntake(outtake5, false),
                shootingSequence(),

                followPath(leave, false),
                new InstantCommand(() -> {
                    TurretCR.staticLastAutoX = follower.getPose().getY();
                    TurretCR.staticLastAutoY = follower.getPose().getX();
                })
        );

        waitForStart();
        schedule(auto);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            run();
            robot.flywheel.loopAuto(2300);
            robot.hoodServo.setPosition(0.825);
            robot.turret.loopAuto(false, follower.getPose(), 1.25, 142);
            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            TurretCR.staticLastAutoX = follower.getPose().getY();
            TurretCR.staticLastAutoY = follower.getPose().getX();
        }
    }
}