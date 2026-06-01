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

@Autonomous(name = "AUTO NEAR 12 ❌ GATE RED FULL CMD❤️")
public class upperRedCMD_12_NOGATE extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double loopTime = 0;
    private final Pose startPose = new Pose(122.99, 124.01, Math.toRadians(36));
    private final Pose outtake = new Pose(85.438, 87.21, Math.toRadians(0));
    private final Pose alignToBalls1 = new Pose(93.91, 87.21, Math.toRadians(0));
    private final Pose intake1 = new Pose(122, 87.21, Math.toRadians(0));
    private final Pose alignToBalls2 = new Pose(98.91, 60, Math.toRadians(0));
    private final Pose intake2 = new Pose(122, 60, Math.toRadians(0));
    private final Pose alignToBalls3 = new Pose(92, 37, Math.toRadians(0));
    private final Pose intake3 = new Pose(122, 37, Math.toRadians(0));
    private final Pose leavePoint = new Pose(100.57, 83.57, Math.toRadians(90));
    private PathChain launchPreload, align1, intaking1, outtaking1, align2, intaking2, outtaking2, align3, intaking3, outtaking3, leave;
    private void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();
        // Sequence for Ball 2
        align2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls2))
                .setConstantHeadingInterpolation(alignToBalls2.getHeading())
                .build();
        intaking2 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls2, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
        outtaking2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, new Pose(60.75, 68.09), outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        // Sequence for Ball 1
        align1 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls1))
                .setConstantHeadingInterpolation(alignToBalls1.getHeading())
                .build();
        intaking1 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls1, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .build();
        outtaking1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        // Sequence for Ball 3
        align3 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls3))
                .setConstantHeadingInterpolation(alignToBalls3.getHeading())
                .build();
        intaking3 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls3, intake3))
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();
        outtaking3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(outtake, leavePoint))
                .setConstantHeadingInterpolation(leavePoint.getHeading())
                .build();
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

    private Command followPathWithAutomation(PathChain path) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(this::isRobotFull).withTimeout(2000),
                        new InstantCommand(() -> follower.followPath(path, true))
                ),
                new WaitCommand(150),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitCommand(150),
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

    private Command shootingSequence() {
        return new SequentialCommandGroup(
                new WaitCommand(500),
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
                new WaitCommand(1500),
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
                followPath(launchPreload, false),
                shootingSequence(),

                followPath(align2, false),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                followPathWithAutomation(intaking2),
                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                new SetIntakeState(Intake.IntakeState.ON),
                new WaitCommand(200),
                new SetIntakeState(Intake.IntakeState.OFF),
                followPath(outtaking2, false),
                shootingSequence(),

                followPath(align1, false),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                followPathWithAutomation(intaking1),
                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                new SetIntakeState(Intake.IntakeState.ON),
                new WaitCommand(200),
                new SetIntakeState(Intake.IntakeState.OFF),
                followPath(outtaking1, false),
                shootingSequence(),

                followPath(align3, false),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                followPathWithAutomation(intaking3),
                new SetServoIntakeState(Intake.ServoIntakeState.UP),
                new SetIntakeState(Intake.IntakeState.ON),
                new WaitCommand(200),
                new SetIntakeState(Intake.IntakeState.OFF),
                followPath(outtaking3, false),
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
            robot.flywheel.loopAuto(1750);
            robot.hoodServo.setPosition(0.575);
            robot.turret.loopAuto(false, follower.getPose(), 144, 140);
            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            TurretCR.staticLastAutoX = follower.getPose().getY();
            TurretCR.staticLastAutoY = follower.getPose().getX();
        }
    }
}