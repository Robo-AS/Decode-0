package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Bottom Blue w Gate Auto (Unified)")
public class bottomBlueAutoWithGate extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private boolean reachedEnd = false;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose outtake = new Pose(56, 18, Math.toRadians(90));
    private final Pose gateControl = new Pose(66.278, 78.064);
    private final Pose openGate = new Pose(17, 71.744);
    private final Pose leaveGateControl = new Pose(45.609, 76.185);
    private final Pose leaveGate = new Pose(54.320, 56.370);
    private final Pose intake1 = new Pose(42, 35, Math.toRadians(180));
    private final Pose intake2 = new Pose(42, 60, Math.toRadians(180));

    private PathChain launchPreload, gate, goToIntake, get1, throw1, get2, throw2;

    public void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierCurve(outtake, gateControl, openGate))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        goToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, leaveGateControl, leaveGate))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        get1 = follower.pathBuilder()
                .addPath(new BezierLine(leaveGate, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        throw1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        get2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        throw2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                follower.followPath(launchPreload);
                setPathState(1);
                break;
            }
            case 1: {
                if (!follower.isBusy()) {
                    follower.followPath(gate, true);
                    setPathState(2);
                }
                break;
            }
            case 2: {
                if (!follower.isBusy()) {
                    follower.followPath(goToIntake, true);
                    setPathState(3);
                }
                break;
            }
            case 3: {
                if (!follower.isBusy()) {
                    follower.followPath(get1, true);
                    setPathState(4);
                }
                break;
            }
            case 4: {
                if (!follower.isBusy()) {
                    follower.followPath(throw1, true);
                    setPathState(5);
                }
                break;
            }
            case 5: {
                if (!follower.isBusy()) {
                    follower.followPath(get2, true);
                    setPathState(6);
                }
                break;
            }
            case 6: {
                if (!follower.isBusy()) {
                    follower.followPath(throw2, true);
                    setPathState(7);
                }
                break;
            }
            case 7: {
                if(!follower.isBusy())
                    reachedEnd = true;
                break;
            }
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

        telemetry.addData("pathState", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}