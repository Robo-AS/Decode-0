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

import java.lang.annotation.Target;

@Autonomous(name = "AUTO MEET JOS ROSU")
public class bottomRedAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double y_distance, x_distance, distance, ty, tx, CAMERA_HEIGHT = 0.4, CAMERA_ANGLE = 18, pos;
    public double downY = 0.1, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    public double TARGET_ANGLE = 19;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private boolean reachedEnd = false;

    private int pathState = 0;
    private int pathSubState = 0;
    private double loopTime = 0;
    private Timer pathTimer = new Timer(), actionTimer = new Timer();

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose outtake   = new Pose(88, 18, Math.toRadians(90));
    private final Pose intake2   = new Pose(77, 32, Math.toRadians(0));
    private final Pose loaded2   = new Pose(110, 32, Math.toRadians(0));
    private final Pose leavePoint   = new Pose(77, 42, Math.toRadians(0));

    private PathChain launchPreload, get2, throw2, loading2, leave;
    public void buildPaths()
    {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        get2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intake2))
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

        leave = follower.pathBuilder()
                .addPath(new BezierLine(outtake, leavePoint))
                .setConstantHeadingInterpolation(leavePoint.getHeading())
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
                robot.servoLauncher.setPosition(0);
                startWait();
                pathSubState = 2;
                break;
            case 2:
                if (!hasWaitElapsed(500)) break;
                robot.servoLauncher.setPosition(0.8);
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
                if (!hasWaitElapsed(450)) break;
                robot.intake.setPower(0);
                startWait();
                pathSubState = 5;
                break;
            case 5:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(0);
                startWait();
                pathSubState = 6;
                break;
            case 6:
                if (!hasWaitElapsed(500)) break;
                robot.servoLauncher.setPosition(0.8);
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
                if (!hasWaitElapsed(450)) break;
                robot.intake.setPower(0);
                startWait();
                pathSubState = 9;
                break;
            case 9:
                if (!hasWaitElapsed(300)) break;
                robot.servoLauncher.setPosition(0);
                startWait();
                pathSubState = 10;
                break;
            case 10:
                if (!hasWaitElapsed(500)) break;
                robot.servoLauncher.setPosition(0.8);

                pathSubState = 11;

                break;
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case -2:
                follower.followPath(launchPreload);
                pathState = -1;
                pathSubState = 0;
                startWait();

                break;

            case -1:
                if(!hasWaitElapsed(2000)) break;

                pathState = 0;
                break;

            case 0:
                if(follower.isBusy()) break;
                shootingSequence();

                if (pathSubState <= 10) break;

                follower.followPath(get2, true);

                pathSubState = 0;
                pathState = 1;

                break;

            case 1:
                if (follower.isBusy()) break;

                robot.intake.setPower(1);

                follower.followPath(loading2, true);
                pathState = 2;

                break;

            case 2:
                if (follower.isBusy()) break;

                robot.intake.setPower(0);
                follower.followPath(throw2, true);
                pathState = 3;

                break;

            case 3:
                if(follower.isBusy()) break;

                startWait();
                pathState = 4;

                break;

            case 4:
                if(!hasWaitElapsed(2000)) break;

                pathState = 5;

                break;

            case 5:
                if(follower.isBusy()) break;

                shootingSequence();

                if(pathSubState <= 10) break;

                pathSubState = 0;
                TARGET_ANGLE = 0;
                follower.followPath(leave);
                pathState = 6;
            case 6:
                if(follower.isBusy()) break;

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

        robot.flywheel.loopAuto(3100);
        robot.servoY.setPosition(0.55);

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
        setPathState(-2);
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