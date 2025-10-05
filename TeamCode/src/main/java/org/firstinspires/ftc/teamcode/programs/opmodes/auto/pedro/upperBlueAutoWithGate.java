package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro.AutoPaths;

@Autonomous(name = "Upper Blue w Gate Auto")
public class upperBlueAutoWithGate extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;

    public static Pose startPose = new Pose(56.000, 135.400, Math.toRadians(90));
    public static Pose outtakePreload = new Pose(32.626, 110.690);
    public static Pose outtake1 = new Pose(50.050, 93.267);
    public static Pose outtake2 = new Pose(62.520, 80.968);

    public static Pose openGate = new Pose(17.000, 72.000);
    public static Pose gateControl = new Pose(66.619, 69.865);
    public static Pose leaveGateControl = new Pose(61.495, 69.181);
    public static Pose leaveGate = new Pose(48.171, 84.384);

    public static Pose intake1 = new Pose(41.000, 84.043);
    public static Pose intake2 = new Pose(41.000, 60.000);

    @Override
    public void runOpMode() {
        robot.initializeHardware(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        robot.initialize();

        follower = Constants.createFollower(Robot.getInstanceHardwareMap());
        follower.setStartingPose(startPose);

        PathBuilder builder = new PathBuilder(follower, new PathConstraints(0, 0));

        PathChain launchPreload = builder.addPath(new BezierLine(startPose, outtakePreload))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        PathChain gate = builder.addPath(new BezierCurve(outtakePreload, gateControl, openGate))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
                .build();

        PathChain goToIntake = builder.addPath(new BezierCurve(openGate, leaveGateControl, leaveGate))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        PathChain get1 = builder.addPath(new BezierLine(leaveGate, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        PathChain throw1 = builder.addPath(new BezierLine(intake1, outtake1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        PathChain get2 = builder.addPath(new BezierLine(outtake1, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        PathChain throw2 = builder.addPath(new BezierLine(intake2, outtake2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        AutoPaths paths = AutoPaths.getInstance();
        paths.resetAll();

        PathChain currentPath = null;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();

            // Only start a path if not already started
            if (!paths.LAUNCH_PRELOAD_COMPLETED) {
                if (currentPath != launchPreload) {
                    follower.followPath(launchPreload, true);
                    currentPath = launchPreload;
                }
                if (!follower.isBusy()) {
                    paths.LAUNCH_PRELOAD_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.GATE_COMPLETED) {
                if (currentPath != gate) {
                    follower.followPath(gate, false);
                    currentPath = gate;
                }
                if (!follower.isBusy()) {
                    paths.GATE_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.GO_TO_INTAKE_COMPLETED) {
                if (currentPath != goToIntake) {
                    follower.followPath(goToIntake, false);
                    currentPath = goToIntake;
                }
                if (!follower.isBusy()) {
                    paths.GO_TO_INTAKE_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.GET1_COMPLETED) {
                if (currentPath != get1) {
                    follower.followPath(get1, false);
                    currentPath = get1;
                }
                if (!follower.isBusy()) {
                    paths.GET1_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.THROW1_COMPLETED) {
                if (currentPath != throw1) {
                    follower.followPath(throw1, false);
                    currentPath = throw1;
                }
                if (!follower.isBusy()) {
                    paths.THROW1_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.GET2_COMPLETED) {
                if (currentPath != get2) {
                    follower.followPath(get2, false);
                    currentPath = get2;
                }
                if (!follower.isBusy()) {
                    paths.GET2_COMPLETED = true;
                    currentPath = null;
                }
            } else if (!paths.THROW2_COMPLETED) {
                if (currentPath != throw2) {
                    follower.followPath(throw2, false);
                    currentPath = throw2;
                }
                if (!follower.isBusy()) {
                    paths.THROW2_COMPLETED = true;
                    currentPath = null;
                }
            }

            // Telemetry for debugging
            telemetry.addData("LaunchPreload", paths.LAUNCH_PRELOAD_COMPLETED);
            telemetry.addData("Gate", paths.GATE_COMPLETED);
            telemetry.addData("GoToIntake", paths.GO_TO_INTAKE_COMPLETED);
            telemetry.addData("Get1", paths.GET1_COMPLETED);
            telemetry.addData("Throw1", paths.THROW1_COMPLETED);
            telemetry.addData("Get2", paths.GET2_COMPLETED);
            telemetry.addData("Throw2", paths.THROW2_COMPLETED);
            telemetry.update();
        }
    }
}
