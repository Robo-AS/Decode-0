package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET SUS SUB POARTA RED")
public class upperRedAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    public double TARGET_ANGLE = 9;

    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 4000;

    private boolean reachedEnd = false;
    private int pathState = 0;
    private int pathSubState = 0;

    private final Pose startPose = new Pose(21.01067615658363, 19.98576512455516, Math.toRadians(216));
    private final Pose outtake = new Pose(50.562, 51.246, Math.toRadians(225));
    private final Pose alignToBalls1 = new Pose(38.09252669039146, 50.7864768683274, Math.toRadians(180));
    private final Pose intake1 = new Pose(9, 50.7864768683274, Math.toRadians(180));
    private final Pose alignToBalls2 = new Pose(38.09252669039146, 77.7864768683274, Math.toRadians(180));
    private final Pose intake2 = new Pose(7, 77.7864768683274, Math.toRadians(180));
    private final Pose leavePoint = new Pose(31.430604982206404, 60.4270462633452, Math.toRadians(180));

    private PathChain launchPreload, align1, intaking1, outtaking1, align2, intaking2, outtaking2, leave;

    private void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
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

        align2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls2))
                .setConstantHeadingInterpolation(alignToBalls2.getHeading())
                .build();

        intaking2 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls2, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        outtaking2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, new Pose(8.048, 110.611), outtake))
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

    private void startWait() {
        waitTimer.resetTimer();
    }

    private boolean hasWaitElapsed(long ms) {
        return waitTimer.getElapsedTime() >= ms;
    }

    private void shootingSequence() {
        switch (pathSubState) {
            case 0:
                startWait();
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                pathSubState = 1;
                break;
            case 1:
                if (!hasWaitElapsed(150)) break;
                robot.servoBarrier.setPosition(0.5);
                startWait();
                pathSubState = 2;
                break;
            case 2:
                if (!hasWaitElapsed(1500)) break;
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
                pathState = 1;
                pathSubState = 0;
                break;

            case 1:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                follower.followPath(align1);
                pathState = 2;
                break;

            case 2:
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                follower.followPath(intaking1, true);
                pathTimeoutTimer.resetTimer();
                pathState = 3;
                break;

            case 3:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 4;
                    break;
                }
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(0);
                follower.followPath(outtaking1, true);
                pathState = 4;
                break;

            case 4:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                follower.followPath(align2);
                pathState = 5;
                break;

            case 5:
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                follower.followPath(intaking2);
                pathTimeoutTimer.resetTimer();
                pathState = 6;
                break;

            case 6:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 7;
                    break;
                }
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(0);
                follower.followPath(outtaking2);
                pathState = 7;
                break;

            case 7:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                TARGET_ANGLE = 0;

                follower.followPath(leave);
                pathState = 8;
                break;

            case 8:
                if (follower.isBusy()) break;
                reachedEnd = true;
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
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimeoutTimer.resetTimer();
        pathState = 0;
        pathSubState = 0;
    }

    @Override
    public void loop() {
        if (reachedEnd) return;
        follower.update();
        autonomousPathUpdate();

        robot.flywheel.loopAuto(1800);
        robot.servoY.setPosition(0.5);
        robot.turret.loopAuto(TARGET_ANGLE);

        robot.x = follower.getPose().getX();
        robot.y = follower.getPose().getY();
        robot.heading = follower.getPose().getHeading();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Sub State", pathSubState);
        telemetry.update();
    }

    @Override
    public void stop() {}
}