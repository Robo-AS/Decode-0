package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET JOS ROSU")
public class bottomRedAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    public double TARGET_ANGLE = -23;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 4000;
    private boolean reachedEnd = false;

    private int pathState = 0;
    private int pathSubState = 0;
    private Timer pathTimer = new Timer();

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(88.000, 18.000, Math.toRadians(90));
    private final Pose intake1 = new Pose(143.500, -3, Math.toRadians(0));
    private final Pose intake2 = new Pose(105.5, 34.5, Math.toRadians(0));
    private final Pose loaded2 = new Pose(145, 34.5, Math.toRadians(0));
    private final Pose leavePoint = new Pose(114, 18, Math.toRadians(0));

    private PathChain launchPreload, get1, get2, throw2, loading2, leave, backFromIntake1;

    public void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        get2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        loading2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, loaded2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        throw2 = follower.pathBuilder()
                .addPath(new BezierLine(loaded2, outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();

        get1 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .build();

        backFromIntake1 = follower.pathBuilder()
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
            case -1:
                follower.followPath(launchPreload);
                pathState = 0;
                pathSubState = 0;
                startWait();
                break;

            case 0:
                if (follower.isBusy()) break;
                if(!hasWaitElapsed(1000)) break;
                shootingSequence();
                if (pathSubState <= 2) break;

                pathSubState = 0;
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get2, true);
                pathState = 1;
                break;

            case 1:
                if (follower.isBusy()) break;
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(loading2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 2;
                break;

            case 2:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.intakeBack.setPower(1);
                    robot.intakeFront.setPower(1);
                    follower.followPath(buildExitPath());
                    pathState = 3;
                    break;
                }
                if (follower.isBusy()) break;
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(throw2, true);
                pathState = 3;
                break;

            case 3:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;

                pathSubState = 0;
                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                follower.followPath(get1, true);
                pathState = 4;
                break;

            case 4:
                if (follower.getCurrentTValue() >= 0.8) {
                    robot.intakeBack.setPower(1);
                    robot.intakeFront.setPower(1);
                }
                if (!follower.isBusy()) {
                    follower.followPath(backFromIntake1, true);
                    pathState = 5;
                }

                robot.intakeBack.setPower(1);
                robot.intakeFront.setPower(1);
                break;

            case 5:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;

                pathSubState = 0;
                TARGET_ANGLE = 0;
                follower.followPath(leave);
                pathState = 6;
                break;

            case 6:
                if (follower.isBusy()) break;
                reachedEnd = true;
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (!reachedEnd) {
            follower.update();
            autonomousPathUpdate();
        }
        robot.flywheel.loopAuto(2300);
        robot.servoY.setPosition(1);
        robot.turret.loopAuto(TARGET_ANGLE);
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
        setPathState(-1);
    }

    @Override
    public void stop() {}
}