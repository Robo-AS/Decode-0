package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.commandbase.auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET JOS ALBASTRU")
public class bottomBlueAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double y_distance, x_distance, distance, ty, tx, CAMERA_HEIGHT = 0.4, CAMERA_ANGLE = 18, pos;
    public double downY = 0.1, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private boolean reachedEnd = false;

    private int pathState = 0;
    private int pathSubState = 0;
    private double loopTime = 0;
    private Timer pathTimer, actionTimer;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(56.000, 18.000, Math.toRadians(90));

    private final Pose intake1 = new Pose(42.000, 35.000, Math.toRadians(180));
    private final Pose loaded1 = new Pose(23.000, 35.000, Math.toRadians(180));
    private final Pose intake2 = new Pose(42.000, 60.000, Math.toRadians(180));
    private final Pose loaded2 = new Pose(23, 60, Math.toRadians(180));

    private PathChain launchPreload, get1, throw1, loading1, get2, throw2, loading2;
    public void buildPaths()
    {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        get1 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .build();

        loading1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, loaded1))
                .setLinearHeadingInterpolation(intake1.getHeading(), loaded1.getHeading())
                .build();

        throw1 = follower.pathBuilder()
                .addPath(new BezierLine(loaded1, outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
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
                shootingSequence();

                if (pathSubState <= 10) break;

                follower.followPath(get1, true);

                pathSubState = 0;
                pathState = 1;

                break;

            case 1:
                if (follower.isBusy()) break;

                robot.intake.setPower(1);

                follower.followPath(loading1, true);
                pathState = 2;

                break;

            case 2:
                if (follower.isBusy()) break;

                robot.intake.setPower(0);
                follower.followPath(throw1, true);
                pathState = 3;

                break;

            case 3:
                if (follower.isBusy()) break;

                shootingSequence();

                if(pathSubState <= 10) break;

                follower.followPath(get2, true);

                pathSubState = 0;
                pathState = 4;

                break;

            case 4:
                if (follower.isBusy()) break;

                robot.intake.setPower(1);

                follower.followPath(loading2, true);
                pathState = 5;
                break;
            case 5:
                if(follower.isBusy()) break;

                robot.intake.setPower(0);

                follower.followPath(throw2, true);
                pathState = 6;
                break;

            case 6:
                if(follower.isBusy()) break;

                shootingSequence();

                if(pathSubState <= 10) break;

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

        robot.turret.loopAuto(-25);
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
        setPathState(0);
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