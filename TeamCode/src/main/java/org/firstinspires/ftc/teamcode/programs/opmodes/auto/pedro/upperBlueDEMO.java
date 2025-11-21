package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO DEMO SUS SUB POARTA BLUE")
public class upperBlueDEMO extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double y_distance, x_distance, distance, ty, tx, CAMERA_HEIGHT = 0.4, CAMERA_ANGLE = 18, pos;
    public double downY = 0.1, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private boolean reachedEnd = false;

    private int pathState = 0;
    private int pathSubState = 0;
    private int throwCycle = 0; // counts 0,1,2 for three shots

    private final Pose startPose = new Pose(21.01067615658363, 124.01423487544484, Math.toRadians(144));
    private final Pose outtake = new Pose(50.562, 92.754, Math.toRadians(135));
    private final Pose alignToBalls1 = new Pose(38.09252669039146, 97, Math.toRadians(180));
    private final Pose intake1 = new Pose(15, 97, Math.toRadians(180));

    private PathChain launchPreload, align1, intaking1, outtaking1;

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
                pathSubState = 1;
                break;
            case 1:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(1);
                startWait();
                pathSubState = 2;
                break;
            case 2:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(0);
                startWait();
                pathSubState = 3;
                break;
            case 3:
                if (!hasWaitElapsed(500)) break;
                robot.intake.setPower(1);
                startWait();
                pathSubState = 4;
                break;
            case 4:
                if (!hasWaitElapsed(700)) break;
                robot.intake.setPower(0);
                startWait();
                pathSubState = 5;
                break;
            case 5:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(1);
                startWait();
                pathSubState = 6;
                break;
            case 6:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(0);
                startWait();
                pathSubState = 7;
                break;
            case 7:
                if (!hasWaitElapsed(500)) break;
                robot.intake.setPower(1);
                startWait();
                pathSubState = 8;
                break;
            case 8:
                if (!hasWaitElapsed(700)) break;
                robot.intake.setPower(0);
                startWait();
                pathSubState = 9;
                break;
            case 9:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(1);
                startWait();
                pathSubState = 10;
                break;
            case 10:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(0);

                pathSubState = 11;

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

                if (pathSubState <= 10) break;

                follower.followPath(align1);
                pathSubState = 0;
                pathState = 2;

                break;

            case 2:
                if (follower.isBusy()) break;

                robot.intake.setPower(1);
                follower.followPath(intaking1, true);
                pathState = 3;

                break;

            case 3:
                if (follower.isBusy()) break;

                robot.intake.setPower(0);
                follower.followPath(outtaking1, true);
                pathState = 4;

                break;

            case 4:
                if (follower.isBusy()) break;

                shootingSequence();

                if (pathSubState <= 10) break;

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
        pathState = 0;
        pathSubState = 0;
        throwCycle = 0;
    }

    @Override
    public void loop() {
        if (reachedEnd) return;
        follower.update();
        autonomousPathUpdate();

        tx = robot.limelight.getLatestResult().getTx();
        ty = robot.limelight.getLatestResult().getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
        distance = Math.sqrt(x_distance*x_distance + y_distance*y_distance);

        robot.flywheel.loop(distance);
        pos = getServoYPositionFromDistance(y_distance);
        robot.servoY.setPosition(pos);
    }

    @Override
    public void stop() {}

    public double getServoYPositionFromDistance(double distance)
    {
        if(distance < maxDistance) return 0.6;
        if(distance > minDistance) return 0.1;

        double ratio = (minDistance - distance) / (minDistance - maxDistance);
        return downY + ratio * (upY - downY);
    }
}