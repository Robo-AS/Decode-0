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

@Autonomous(name = "🏰IST upper blue 🔵")
public class ISTupperBlueAuto extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double loopTime = 0;
    private final Pose startPose = new Pose(21.010, 124.010, Math.toRadians(144));

    private PathChain preload, spike2, outtake1, gate1, outtake2, spike1, outtake3, leave;

    private void buildPaths() {
        preload = follower.pathBuilder().addPath(
                new BezierLine(new Pose(21.010, 124.010), new Pose(60.000, 84.000))
        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(225)).build();

        spike2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60.000, 84.000), new Pose(65.1637010676157, 60.604982206405694), new Pose(18, 59.44))
        ).setTangentHeadingInterpolation().build();

        outtake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(18, 59.44), new Pose(60.000, 84.000))
        ).setTangentHeadingInterpolation().setReversed().build();

        gate1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(60.000, 84.000), new Pose(55.74, 65.80), new Pose(31.66, 64.54)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(31.66, 64.54), new Pose(7.13, 57.25)))
                .setConstantHeadingInterpolation(Math.toRadians(165.1))
                .build();

        outtake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(7.13, 57.25), new Pose(60.000, 84.000))
        ).setTangentHeadingInterpolation().setReversed().build();

        spike1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(60.000, 84.000), new Pose(22.5, 84.000))
        ).setTangentHeadingInterpolation().build();

        outtake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(22.5, 84.000), new Pose(60.000, 84.000))
        ).setTangentHeadingInterpolation().setReversed().build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(new Pose(60.000, 84.000), new Pose(37.189, 83.669))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
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

    private Command followPathWithMidpointIntake(PathChain path) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(this::isRobotFull).withTimeout(2500),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.followPath(path, true)),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() >= 0.3),
                                new SetIntakeState(Intake.IntakeState.ON),
                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                new WaitUntilCommand(() -> !follower.isBusy())
                        )
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
                new WaitCommand(450),
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
                followPath(preload, false),
                shootingSequence(),

                followPathWithMidpointIntake(spike2),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                new WaitCommand(100),
                new SetIntakeState(Intake.IntakeState.OFF),
                followPath(outtake1, false),
                shootingSequence(),

                new SetIntakeState(Intake.IntakeState.ON),
                followPathWithMidpointIntake(gate1),
                new SetIntakeState(Intake.IntakeState.ON),
                new SetServoIntakeState(Intake.ServoIntakeState.AUTO_GATE),
                new WaitCommand(2000),
                new SetIntakeState(Intake.IntakeState.OFF),
                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                followPath(outtake2, false),
                shootingSequence(),

                followPathWithMidpointIntake(spike1),
                new SetIntakeState(Intake.IntakeState.ON),
                new WaitCommand(100),
                new SetIntakeState(Intake.IntakeState.OFF),
                followPath(outtake3, false),
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
            robot.flywheel.loopAuto(1800);
            robot.hoodServo.setPosition(0.65);
            robot.turret.loopAuto(false, follower.getPose(), -2.5, 142);
            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            TurretCR.staticLastAutoX = follower.getPose().getY();
            TurretCR.staticLastAutoY = follower.getPose().getX();
        }
    }
}