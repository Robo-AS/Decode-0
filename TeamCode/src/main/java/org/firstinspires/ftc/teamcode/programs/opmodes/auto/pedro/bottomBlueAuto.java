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

@Autonomous(name = "Bottom Blue Auto WITH SWITCH")
public class bottomBlueAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean reachedEnd = false;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(56.000, 18.000, Math.toRadians(90));

    private final Pose intake1 = new Pose(42.000, 35.000, Math.toRadians(180));
    private final Pose loaded1 = new Pose(23.000, 35.000, Math.toRadians(180));
    private final Pose intake2 = new Pose(42.000, 60.000, Math.toRadians(180));
    private final Pose intake3 = new Pose(42.00, 84.000, Math.toRadians(180));

    private PathChain launchPreload, get1, throw1, get2, throw2, get3, throw3, loading1;
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

        throw2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();

        get3  = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3))
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();

        throw3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, outtake))
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
                    follower.followPath(get1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(loading1,true);
                    robot.intake.setPower(0.75);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(throw1,true);
                    robot.intake.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(get2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(throw2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(get3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(throw3,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
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

       // robot.turret.autoAlignToBlueGoal(follower.getPose());

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
}