package org.firstinspires.ftc.teamcode.programs.opmodes.auto.nodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.commandbase.auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.programs.utils.Node;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Hybrid Upper Blue w Gate Auto")
public class hybridUpperBlueWithGate extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();

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
    private Node currentNode;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(Robot.getInstanceHardwareMap());
        follower.setStartingPose(startPose);

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        PathBuilder builder = new PathBuilder(follower, new PathConstraints(0, 0));

        PathChain launchPreload = builder
                .addPath(new BezierLine(startPose, outtakePreload))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        PathChain gate = builder
                .addPath(new BezierCurve(outtakePreload, gateControl, openGate))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
                .build();

        PathChain goToIntake = builder
                .addPath(new BezierCurve(openGate, leaveGateControl, leaveGate))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        PathChain get1 = builder
                .addPath(new BezierLine(leaveGate, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        PathChain throw1 = builder
                .addPath(new BezierLine(intake1, outtake1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        PathChain get2 = builder
                .addPath(new BezierLine(outtake1, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        PathChain throw2 = builder
                .addPath(new BezierLine(intake2, outtake2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();


        Node n1 = new Node("Preload");
        n1.addConditions(
                () -> new FollowPathCommand(follower, launchPreload, true).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n2 = new Node("Gate");
        n2.addConditions(
                () -> new FollowPathCommand(follower, gate, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n3 = new Node("GoToIntake");
        n3.addConditions(
                () -> new FollowPathCommand(follower, goToIntake, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n4 = new Node("Get1");
        n4.addConditions(
                () -> new FollowPathCommand(follower, get1, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n5 = new Node("Throw1");
        n5.addConditions(
                () -> new FollowPathCommand(follower, throw1, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n6 = new Node("Get2");
        n6.addConditions(
                () -> new FollowPathCommand(follower, get2, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        Node n7 = new Node("Throw2");
        n7.addConditions(
                () -> new FollowPathCommand(follower, throw2, false).schedule(),
                () -> !follower.isBusy(),
                null
        );

        n1.next = new Node[]{n2};
        n2.next = new Node[]{n3};
        n3.next = new Node[]{n4};
        n4.next = new Node[]{n5};
        n5.next = new Node[]{n6};
        n6.next = new Node[]{n7};
        n7.next = new Node[]{};

        currentNode = n1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();
            robot.getInstanceLimelight().loop();

            if (currentNode != null) {
                currentNode.run();
                if (currentNode.transition()) {
                    if (currentNode.next != null && currentNode.next.length > 0) {
                        currentNode = currentNode.next[0];
                    } else {
                        currentNode = null;
                    }
                }
            }

            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Current Node", currentNode != null ? currentNode.name : "Done");
            loopTime = loop;
            telemetry.update();
        }
    }
}
