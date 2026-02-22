package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET JOS ROSU")
public class bottomRedAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private CommandScheduler scheduler;
    private double TARGET_ANGLE = -19;
    private boolean reachedEnd = false;
    private int pathState = 0;
    private int pathSubState = 0;
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 4000;

    // Mirrored Poses (144 - BlueX)
    private final Pose startPose = new Pose(88, 8, Math.toRadians(0));
    private final Pose outtake = new Pose(88.000, 13.000, Math.toRadians(0));
    private final Pose intake1 = new Pose(140, -3, Math.toRadians(0));
    private final Pose intake2 = new Pose(104.000, 32.5, Math.toRadians(0));
    private final Pose intake3 = new Pose(140, 5, Math.toRadians(0));
    private final Pose intake4 = new Pose(140, 30, Math.toRadians(0));
    private final Pose loaded2 = new Pose(136.5, 32.5, Math.toRadians(0));
    private final Pose leavePoint = new Pose(114, 18, Math.toRadians(90));

    private PathChain launchPreload, get1, get2, get3, get4, throw2, loading2, leave, backFromIntake1, backFromIntake3, backFromIntake4;

    public void buildPaths() {
        launchPreload = follower.pathBuilder().addPath(new BezierLine(startPose, outtake)).setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading()).build();

        get2 = follower.pathBuilder().addPath(new BezierLine(outtake, intake2)).setConstantHeadingInterpolation(intake2.getHeading()).build();
        loading2 = follower.pathBuilder().addPath(new BezierLine(intake2, loaded2)).setConstantHeadingInterpolation(intake2.getHeading()).build();
        throw2 = follower.pathBuilder().addPath(new BezierLine(loaded2, outtake)).setConstantHeadingInterpolation(outtake.getHeading()).build();

        get1 = follower.pathBuilder().addPath(new BezierLine(outtake, intake1)).setConstantHeadingInterpolation(intake1.getHeading()).build();
        backFromIntake1 = follower.pathBuilder().addPath(new BezierLine(intake1, outtake)).setConstantHeadingInterpolation(outtake.getHeading()).build();

        get3 = follower.pathBuilder().addPath(new BezierLine(outtake, intake3)).setConstantHeadingInterpolation(intake3.getHeading()).build();
        backFromIntake3 = follower.pathBuilder().addPath(new BezierLine(intake3, outtake)).setConstantHeadingInterpolation(outtake.getHeading()).build();

        get4 = follower.pathBuilder().addPath(new BezierLine(outtake, intake4)).setConstantHeadingInterpolation(intake4.getHeading()).build();
        backFromIntake4 = follower.pathBuilder().addPath(new BezierLine(intake4, outtake)).setConstantHeadingInterpolation(outtake.getHeading()).build();

        leave = follower.pathBuilder().addPath(new BezierLine(outtake, leavePoint)).setConstantHeadingInterpolation(leavePoint.getHeading()).build();
    }

    private PathChain buildExitPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
    }

    private void shootingSequence() {
        switch (pathSubState) {
            case 0:
                waitTimer.resetTimer();
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                pathSubState = 1;
                break;
            case 1:
                if (waitTimer.getElapsedTime() < 150) break;
                robot.servoBarrier.setPosition(0.5);
                waitTimer.resetTimer();
                pathSubState = 2;
                break;
            case 2:
                if (waitTimer.getElapsedTime() < 1500) break;
                robot.servoBarrier.setPosition(0.35);
                robot.intakeBack.setPower(0);
                robot.intakeFront.setPower(0);
                pathSubState = 3;
                break;
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(launchPreload);
                pathState = 0;
                pathSubState = 0;
                waitTimer.resetTimer();
                break;

            case 0: // Preload outtake
                if (follower.isBusy() || waitTimer.getElapsedTime() < 1000) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                // START INTAKE here as we head to the ball
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 1;
                break;

            case 1: // Moving to Ball 2
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 3;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(loading2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 2;
                break;

            case 2: // Loading Ball 2
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 3;
                    break;
                }
                if (follower.isBusy()) break;
                // STOP INTAKE before moving back to outtake
                robot.intakeBack.setPower(0);
                robot.intakeFront.setPower(0);
                follower.followPath(throw2, true);
                pathState = 3;
                break;

            case 3: // Outtaking Ball 2
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                // START INTAKE
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get1, true);
                pathTimeoutTimer.resetTimer();
                pathState = 4;
                break;

            case 4: // Intaking Ball 1
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 5;
                    break;
                }
                if (!follower.isBusy()) {
                    // STOP INTAKE
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(backFromIntake1, true);
                    pathState = 5;
                }
                break;

            case 5: // Outtaking Ball 1
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                // START INTAKE
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get3, true);
                pathTimeoutTimer.resetTimer();
                pathState = 7;
                break;

            case 7: // Intaking Ball 3
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 8;
                    break;
                }
                if (!follower.isBusy()) {
                    // STOP INTAKE
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(backFromIntake3, true);
                    pathState = 8;
                }
                break;

            case 8: // Outtaking Ball 3
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                // START INTAKE
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get4, true);
                pathTimeoutTimer.resetTimer();
                pathState = 9;
                break;

            case 9: // Intaking Ball 4
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 10;
                    break;
                }
                if (!follower.isBusy()) {
                    // STOP INTAKE
                    robot.intakeBack.setPower(0);
                    robot.intakeFront.setPower(0);
                    follower.followPath(backFromIntake4, true);
                    pathState = 10;
                }
                break;

            case 10: // Outtaking Ball 4
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                // Ensure intake is off during parking
                robot.intakeBack.setPower(0);
                robot.intakeFront.setPower(0);
                TARGET_ANGLE = 0;
                follower.followPath(leave);
                pathState = 6;
                break;

            case 6:
                if (follower.isBusy()) break;

                reachedEnd = true;

                TurretCR.staticLastAutoX = follower.getPose().getX();
                TurretCR.staticLastAutoY = follower.getPose().getY();

                break;
        }
    }

    @Override
    public void init() {
        Robot.clearInstance();
        robot = Robot.getInstance();
        scheduler = CommandScheduler.getInstance();
        scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.initializeHardwareAuto(hardwareMap);
        robot.initializeAuto();
        robot.servoBarrier.setPosition(0.35);
    }

    @Override
    public void start() {
        pathState = -1;
        pathTimeoutTimer.resetTimer();
    }

    @Override
    public void loop() {
        scheduler.run();
        if (!reachedEnd) {
            follower.update();
            autonomousPathUpdate();
        }
        robot.flywheel.loopAuto(2300);
        robot.servoY.setPosition(1);
        robot.turret.loopAuto(true, follower.getPose(), 143, 144);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Target Angle", robot.turret.getTargetAngle());
        telemetry.update();
    }
}