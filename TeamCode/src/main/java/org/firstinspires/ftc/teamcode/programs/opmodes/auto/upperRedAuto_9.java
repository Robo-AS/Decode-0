package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO TRIUNGHI MARE 9 RED")
public class upperRedAuto_9 extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 2500;
    private boolean reachedEnd = false;
    private int pathState = 0;
    private int pathSubState = 0;

    // Mirrored Poses
    private final Pose startPose = new Pose(122.99, 124.01, Math.toRadians(36));
    private final Pose outtake = new Pose(93.438, 92.754, Math.toRadians(0));
    private final Pose alignToBalls1 = new Pose(99.91, 82.21, Math.toRadians(0));
    private final Pose intake1 = new Pose(126, 82.21, Math.toRadians(0));
    private final Pose alignToBalls2 = new Pose(98.91, 60, Math.toRadians(0));
    private final Pose intake2 = new Pose(134, 60, Math.toRadians(0));
    private final Pose leavePoint = new Pose(112.57, 83.57, Math.toRadians(90));

    private PathChain launchPreload, align1, intaking1, outtaking1, align2, intaking2, outtaking2, leave;

    private void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        align2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls2))
                .setConstantHeadingInterpolation(alignToBalls2.getHeading())
                .build();
        intaking2 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls2, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
        outtaking2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, new Pose(83.25, 68.09), outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();

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

        leave = follower.pathBuilder()
                .addPath(new BezierLine(outtake, leavePoint))
                .setConstantHeadingInterpolation(leavePoint.getHeading())
                .build();
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
            case 0:
                follower.followPath(launchPreload);
                robot.servoIntake.setPosition(0.45);
                pathState = 1;
                break;
            case 1:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(align2);
                    pathState = 2;
                }
                break;
            case 2:
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 3;
                break;
            case 3:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    follower.followPath(buildExitPath());
                    pathState = 4;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(outtaking2);
                pathState = 4;
                break;
            case 4:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(align1);
                    pathState = 5;
                }
                break;
            case 5:
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking1);
                pathTimeoutTimer.resetTimer();
                pathState = 6;
                break;
            case 6:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    follower.followPath(buildExitPath());
                    pathState = 7;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(outtaking1);
                pathState = 7;
                break;
            case 7:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(leave);
                    pathState = 8;
                }
                break;
            case 8:
                if (follower.isBusy()) break;
                reachedEnd = true;
                TurretCR.staticLastAutoX = follower.getPose().getY();
                TurretCR.staticLastAutoY = follower.getPose().getX();
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.initializeHardwareAuto(hardwareMap);
        robot.initializeAuto();
        robot.servoBarrier.setPosition(0.35);
    }

    @Override
    public void start() {
        pathState = 0;
        pathSubState = 0;
    }

    @Override
    public void loop() {
        if (!reachedEnd) {
            follower.update();
            autonomousPathUpdate();
        }
        robot.flywheel.loopAuto(1700);
        robot.servoY.setPosition(0.75);
        robot.turret.loopAuto(false, follower.getPose(), 146, 144);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Sub State", pathSubState);
        telemetry.update();
    }
}