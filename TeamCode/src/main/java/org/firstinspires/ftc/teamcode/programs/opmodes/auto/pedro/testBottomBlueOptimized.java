package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Test Bottom Blue Optimized")
public class testBottomBlueOptimized extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private boolean reachedEnd = false;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(56.000, 18.000, Math.toRadians(90));

    private PathChain launchPreload, get1, load1, throw1, get2, load2, throw2, get3, load3, throw3;

    public void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        get1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 35.000)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        load1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 35.000), new Pose(23.000, 35.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        throw1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.000, 35.000), new Pose(56.000, 18.000)))
                .setConstantHeadingInterpolation(Math.toRadians(330))
                .build();

        get2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 18.000), new Pose(56.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        load2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 60.000), new Pose(42.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        throw2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(56.000, 18.000)))
                .setConstantHeadingInterpolation(Math.toRadians(290))
                .build();

        get3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 18.000), new Pose(56.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        load3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 84.000), new Pose(42.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        throw3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.000, 84.000), new Pose(56.000, 18.000)))
                .setConstantHeadingInterpolation(Math.toRadians(280))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(launchPreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(get1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(load1, true);
                    robot.intake.setPower(0.75);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(throw1, true);
                    robot.intake.setPower(0);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(get2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(load2, true);
                    robot.intake.setPower(0.75);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(throw2, true);
                    robot.intake.setPower(0);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(get3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(load3, true);
                    robot.intake.setPower(0.75);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(throw3, true);
                    robot.intake.setPower(0);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    reachedEnd = true;
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (reachedEnd) return;

        follower.update();
        autonomousPathUpdate();

       // robot.turret.autoAlignToBlueGoal(follower.getPose());

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
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

        robot.initializeHardwareAuto(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    }

    @Override public void init_loop() {}
    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
    @Override public void stop() {}
}
