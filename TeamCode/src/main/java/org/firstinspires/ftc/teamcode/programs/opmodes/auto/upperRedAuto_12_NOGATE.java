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

@Autonomous(name = "AUTO TRIUNGHI MARE 12 no gate RED")
public class upperRedAuto_12_NOGATE extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 2500;
    private boolean reachedEnd = false;
    private int pathState = 0;
    private int pathSubState = 0;

    private final Pose startPose = new Pose(122.99, 124.01, Math.toRadians(36));
    private final Pose outtake = new Pose(93.438, 92.754, Math.toRadians(0));
    private final Pose alignToBalls1 = new Pose(99.91, 87.21, Math.toRadians(0));
    private final Pose intake1 = new Pose(126, 87.21, Math.toRadians(0));
    private final Pose alignToBalls2 = new Pose(98.91, 60, Math.toRadians(0));
    private final Pose intake2 = new Pose(134, 60, Math.toRadians(0));
    private final Pose alignToBalls3 = new Pose(92, 37, Math.toRadians(0));
    private final Pose intake3 = new Pose(126, 37, Math.toRadians(0));
    private final Pose leavePoint = new Pose(112.57, 83.57, Math.toRadians(90));

    private PathChain launchPreload, align1, intaking1, outtaking1, align2, intaking2, outtaking2, align3, intaking3, outtaking3, leave;

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

        align3 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, alignToBalls3))
                .setConstantHeadingInterpolation(alignToBalls3.getHeading())
                .build();
        intaking3 = follower.pathBuilder()
                .addPath(new BezierLine(alignToBalls3, intake3))
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();
        outtaking3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3, new Pose(79, 50), outtake))
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
            case 0: // Preload
                follower.followPath(launchPreload);
                robot.servoIntake.setPosition(0.45);
                pathState = 1;
                waitTimer.resetTimer();
                break;
            case 1: // Shoot Preload
                if (follower.isBusy() || waitTimer.getElapsedTime() < 500) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(align2);
                    pathState = 2;
                }
                break;
            case 2: // Align Ball 2
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 3;
                break;
            case 3: // Intake Ball 2
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    follower.followPath(buildExitPath());
                    pathState = 4;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(outtaking2);
                pathState = 4;
                waitTimer.resetTimer();
                break;
            case 4: // Shoot Ball 2
                if (follower.isBusy() || waitTimer.getElapsedTime() < 500) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(align1);
                    pathState = 5;
                }
                break;
            case 5: // Align Ball 1
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking1);
                pathTimeoutTimer.resetTimer();
                pathState = 6;
                break;
            case 6: // Intake Ball 1
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    follower.followPath(buildExitPath());
                    pathState = 7;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(outtaking1);
                pathState = 7;
                waitTimer.resetTimer();
                break;
            case 7: // Shoot Ball 1
                if (follower.isBusy() || waitTimer.getElapsedTime() < 500) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(align3);
                    pathState = 8;
                }
                break;
            case 8: // Align Ball 3
                if (follower.isBusy()) break;
                robot.intakeFront.setPower(1);
                robot.intakeBack.setPower(1);
                follower.followPath(intaking3, true);
                pathTimeoutTimer.resetTimer();
                pathState = 9;
                break;
            case 9: // Intake Ball 3
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    follower.followPath(buildExitPath());
                    pathState = 10;
                    break;
                }
                if (follower.isBusy()) break;
                follower.followPath(outtaking3);
                pathState = 10;
                waitTimer.resetTimer();
                break;
            case 10: // Shoot Ball 3
                if (follower.isBusy() || waitTimer.getElapsedTime() < 500) break;
                shootingSequence();
                if (pathSubState == 3) {
                    pathSubState = 0;
                    follower.followPath(leave);
                    pathState = 11;
                }
                break;
            case 11: // Finish
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
        robot.hoodServo.setPosition(0.7);
        robot.turret.loopAuto(false, follower.getPose(), 144, 144);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Sub State", pathSubState);
        telemetry.update();
    }
}