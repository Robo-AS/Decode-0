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

@Autonomous(name = "AUTO DEMO SUS SUB POARTA BLUE")
public class upperBlueForDEMO extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private double y_distance, x_distance, distance, ty, tx, CAMERA_HEIGHT = 0.4, CAMERA_ANGLE = 18;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean reachedEnd = false;
    private boolean waiting = false;
    private long waitDuration = 0;
    private long waitStartTime = 0;


    private int pathState;

    private final Pose startPose = new Pose(21.01067615658363, 124.01423487544484, Math.toRadians(144));
    private final Pose outtake = new Pose(50.562, 92.754, Math.toRadians(135));
    private final Pose alignToBalls1 = new Pose(42.192, 83.530, Math.toRadians(180));
    private final Pose intake1 = new Pose(22.548, 83.530, Math.toRadians(180));

    private PathChain launchPreload, align1, intaking1, outtaking1;
    public void buildPaths()
    {
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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(launchPreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //give the limelight time to focus for flywheel
                    resetWait();
                    if(!WaitCommand(1000)) break;

                    //artifact 1
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    //move the other two up
                    robot.intake.setPower(1);

                    //artifact 2
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    //artifact 3
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    robot.intake.setPower(0);

                    follower.followPath(align1,true);

                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    robot.intake.setPower(1);
                    follower.followPath(intaking1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    robot.intake.setPower(0);
                    follower.followPath(outtaking1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    resetWait();
                    if(!WaitCommand(1000)) break;

                    //artifact 1
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    //move the other two up
                    robot.intake.setPower(1);

                    //artifact 2
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    //artifact 3
                    robot.servoLauncher.setPosition(1);
                    resetWait();
                    if(!WaitCommand(100)) break;
                    robot.servoLauncher.setPosition(0);

                    robot.intake.setPower(0);

                    reachedEnd = true;
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if(reachedEnd)
            return;

        follower.update();
        autonomousPathUpdate();

        tx = robot.limelight.getLatestResult().getTx();
        ty = robot.limelight.getLatestResult().getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx));
        distance = Math.sqrt(x_distance*x_distance + y_distance*y_distance);
        robot.flywheel.loop(distance);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        robot.initializeHardwareAuto(hardwareMap);
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

    public boolean WaitCommand(long ms) {
        if (!waiting) {
            waiting = true;
            waitDuration = ms;
            waitStartTime = System.currentTimeMillis();
            return false;
        }

        long elapsed = System.currentTimeMillis() - waitStartTime;

        if (elapsed >= waitDuration) {
            waiting = false;
            return true;
        }

        return false;
    }

    private void resetWait() {
        waiting = false;
    }
}