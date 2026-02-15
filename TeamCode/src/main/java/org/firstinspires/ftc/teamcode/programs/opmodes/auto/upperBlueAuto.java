package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET SUS SUB POARTA BLUE")
public class upperBlueAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    public double TARGET_ANGLE = 0;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 4000;

    private boolean reachedEnd = false;
    private int pathState = 0;
    private int pathSubState = 0;

    private final Pose startPose = new Pose(21.01067615658363, 124.01423487544484, Math.toRadians(144));
    private final Pose outtake = new Pose(50.562, 92.754, Math.toRadians(135));
    private final Pose alignToBalls1 = new Pose(44.09252669039146, 82.2135231316726, Math.toRadians(180));
    private final Pose intake1 = new Pose(18, 82.2135231316726, Math.toRadians(180));
    private final Pose alignToBalls2 = new Pose(45.09252669039146, 52, Math.toRadians(180));
    private final Pose intake2 = new Pose(10, 52, Math.toRadians(180));
    private final Pose alignToBalls3 = new Pose(52, 32, Math.toRadians(180));
    private final Pose intake3 = new Pose(20, 32, Math.toRadians(180));
    private final Pose leavePoint = new Pose(31.430604982206404, 83.5729537366548, Math.toRadians(180));
    private final Pose openGate = new Pose(11, 57, Math.toRadians(180));

    private PathChain launchPreload, align1, intaking1, outtaking1, align2, intaking2, outtaking2, leave, openDaGate, align3, intaking3, outtaking3;

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

        outtaking2 = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, new Pose(60.75786832740214, 68.09408185053384), outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(outtake, leavePoint))
                .setConstantHeadingInterpolation(leavePoint.getHeading())
                .build();

        openDaGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake2, new Pose(44.188473309608554, 53.91614590747334), openGate))
                .setConstantHeadingInterpolation(openGate.getHeading())
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
                robot.servoBarrier.setPosition(0.48);
                startWait();
                pathSubState = 2;
                break;
            case 2:
                if (!hasWaitElapsed(2000)) break;
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
                pathSubState = 0;
                break;
            case 1:
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                follower.followPath(align2);
                pathState = 2;
                break;
            case 2:
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 3;
                break;
            case 3:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.servoIntake.setPosition(0.45);
                    follower.followPath(buildExitPath());
                    pathState = 4;
                    break;
                }
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                follower.followPath(openDaGate, true);
                pathState = -1;
                break;
            case -1:
                if(follower.isBusy()) break;
                follower.followPath(outtaking2);
                pathState = 4;
                break;
            case 4:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                pathSubState = 0;
                follower.followPath(align1);
                pathState = 5;
                break;
            case 5:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                follower.followPath(intaking1);
                pathTimeoutTimer.resetTimer();
                pathState = 6;
                break;
            case 6:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.servoIntake.setPosition(0.45);
                    follower.followPath(buildExitPath());
                    pathState = 7;
                    break;
                }
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                follower.followPath(outtaking1);
                pathState = 7;
                break;
            case 7:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                robot.intakeFront.setPower(0);
                robot.intakeBack.setPower(0);
                pathSubState = 0;
                follower.followPath(align3);
                pathState = 8;
                break;
            case 8:
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking3, true);
                pathTimeoutTimer.resetTimer();
                pathState = 9;
                break;
            case 9:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    robot.servoIntake.setPosition(0.45);
                    follower.followPath(buildExitPath());
                    pathState = 10;
                    break;
                }
                if (follower.isBusy()) break;
                robot.servoIntake.setPosition(0.45);
                follower.followPath(outtaking3);
                pathState = 10;
                break;
            case 10:
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                if (follower.isBusy()) break;
                shootingSequence();
                if (pathSubState <= 2) break;
                robot.intakeFront.setPower(0);
                robot.intakeBack.setPower(0);
                pathSubState = 0;
                follower.followPath(leave);
                pathState = 11;
                break;
            case 11:
                if (follower.isBusy()) break;
                reachedEnd = true;
                pathState = 12;
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
        robot.servoBarrier.setPosition(0.425);
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
        robot.turret.loopAuto(2);
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