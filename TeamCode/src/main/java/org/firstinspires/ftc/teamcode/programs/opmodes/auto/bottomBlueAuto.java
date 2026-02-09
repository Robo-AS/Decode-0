package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

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
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "AUTO MEET JOS ALBASTRU")
public class bottomBlueAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private double y_distance, x_distance, distance, ty, tx, CAMERA_HEIGHT = 0.4, CAMERA_ANGLE = 18, pos;
    public double downY = 0.1, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    public double TARGET_ANGLE = 30.5;
    private Timer opmodeTimer = new Timer();
    private Timer waitTimer = new Timer();
    private Timer pathTimeoutTimer = new Timer();
    private final long PATH_TIMEOUT_MS = 4000;
    private boolean reachedEnd = false;

    private int pathState = 0;
    private int pathSubState = 0;
    private double loopTime = 0;
    private Timer pathTimer = new Timer(), actionTimer = new Timer();

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(56.000, 18.000, Math.toRadians(90));
    private final Pose intake1 = new Pose(8, 8, Math.toRadians(180));
    private final Pose intake2 = new Pose(40.000, 35.5, Math.toRadians(180));
    private final Pose loaded2 = new Pose(17, 35.5, Math.toRadians(180));
    private final Pose leavePoint = new Pose(30, 18, Math.toRadians(180));

    private PathChain launchPreload, get1, get2, throw2, loading2, leave;
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
                if(follower.isBusy()) break;
                shootingSequence();

                if (pathSubState <= 2) break;
                pathSubState = 0;

                follower.followPath(get1, true);

                pathState = -2;

                break;

            case -2:
                if (follower.getCurrentTValue() >= 0.8) {
                    robot.intakeFront.setPower(1);
                }

                if (!follower.isBusy()) {
                    follower.followPath(buildExitPath(), true);
                    pathState = -3;
                }
                break;

            case -3:
                if (follower.isBusy()) break;

                shootingSequence();

                if(pathSubState <= 2) break;

                pathSubState = 0;

                follower.followPath(get2, true);

                pathState = 1;
                break;

            case 1:
                if (follower.isBusy()) break;

                robot.intakeFront.setPower(1);
                //aici ridici bariera

                follower.followPath(loading2, true);
                pathTimeoutTimer.resetTimer();
                pathState = 2;

                break;

            case 2:
                if (follower.isBusy() && pathTimeoutTimer.getElapsedTime() >= PATH_TIMEOUT_MS) {
                    telemetry.addData("Failsafe", "Intaking 1 Timeout. Running EXIT.");
                    robot.intakeFront.setPower(0);
                    follower.followPath(buildExitPath());
                    pathState = 3;
                    break;
                }

                if (follower.isBusy()) break;

                robot.intakeFront.setPower(0);
                //aici cobori bariera
                follower.followPath(throw2, true);
                pathState = 3;

                break;

            case 3:
                if(follower.isBusy()) break;

                shootingSequence();

                if(pathSubState <= 2) break;

                pathSubState = 0;
                TARGET_ANGLE = 0;

                follower.followPath(leave);

                pathState = 4;

                break;

            case 4:
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