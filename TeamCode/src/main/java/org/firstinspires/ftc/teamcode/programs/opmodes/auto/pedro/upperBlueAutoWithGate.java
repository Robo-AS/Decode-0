package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Upper Blue Auto with Gate")
public class upperBlueAutoWithGate extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private boolean reachedEnd = false;

    private final Pose startPose = new Pose(56.000, 135.400, Math.toRadians(90));
    private final Pose outtakePreload = new Pose(32.626, 110.690, Math.toRadians(315));
    private final Pose openGate = new Pose(17.000, 72.000, Math.toRadians(270));
    private final Pose gateControl = new Pose(66.619, 69.865, Math.toRadians(270));
    private final Pose leaveGateControl = new Pose(61.495, 69.181, Math.toRadians(270));
    private final Pose leaveGate = new Pose(48.171, 84.384, Math.toRadians(270));

    private final Pose intake1 = new Pose(41.000, 84.043, Math.toRadians(180));
    private final Pose outtake1 = new Pose(50.050, 93.267, Math.toRadians(315));
    private final Pose intake2 = new Pose(41.000, 60.000, Math.toRadians(180));
    private final Pose outtake2 = new Pose(62.520, 80.968, Math.toRadians(315));

    private PathChain launchPreload, gate, leaveGatePath, get1, throw1, get2, throw2;

    public void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtakePreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtakePreload.getHeading())
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierCurve(outtakePreload, gateControl, openGate))
                .setLinearHeadingInterpolation(outtakePreload.getHeading(), openGate.getHeading())
                .build();

        leaveGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, leaveGateControl, leaveGate))
                .setLinearHeadingInterpolation(openGate.getHeading(), leaveGate.getHeading())
                .build();

        get1 = follower.pathBuilder()
                .addPath(new BezierLine(leaveGate, intake1))
                .setLinearHeadingInterpolation(leaveGate.getHeading(), intake1.getHeading())
                .build();

        throw1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake1))
                .setLinearHeadingInterpolation(intake1.getHeading(), outtake1.getHeading())
                .build();

        get2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake1, intake2))
                .setLinearHeadingInterpolation(outtake1.getHeading(), intake2.getHeading())
                .build();

        throw2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake2))
                .setLinearHeadingInterpolation(intake2.getHeading(), outtake2.getHeading())
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
                    follower.followPath(gate, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(leaveGatePath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(get1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(throw1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(get2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(throw2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    reachedEnd = true;
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        robot.initializeHardwareAuto(hardwareMap);
    }

    @Override
    public void loop() {
        if (reachedEnd) return;

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {}
}
