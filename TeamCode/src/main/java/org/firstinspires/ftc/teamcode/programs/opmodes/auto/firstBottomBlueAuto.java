package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Bottom Blue Auto")
public class firstBottomBlueAuto extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();

    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose outtake = new Pose(56.000, 18.000);

    public static Pose intake1 = new Pose(42.000, 35.000);
    public static Pose intake2 = new Pose(42.000, 60.000);


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        follower = Constants.createFollower(Robot.getInstanceHardwareMap());
        follower.setStartingPose(startPose);

        robot.initializeHardware(hardwareMap, new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        ));
        robot.initialize();

        PathBuilder builder = new PathBuilder(follower, new PathConstraints(0, 0));

        PathChain launchPreload = builder
                .addPath(new BezierLine(startPose, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))
                .build();

        PathChain get1 = builder
                .addPath(new BezierLine(outtake, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        PathChain throw1 = builder
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        PathChain get2 = builder
                .addPath(new BezierLine(outtake, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        PathChain throw2 = builder
                .addPath(new BezierLine(intake2, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, launchPreload, true),
                new WaitCommand(500),
                new FollowPathCommand(follower, get1, false),
                new WaitCommand(500),
                new FollowPathCommand(follower, throw1, false),
                new WaitCommand(500),
                new FollowPathCommand(follower, get2, false),
                new WaitCommand(500),
                new FollowPathCommand(follower, throw2, false)
        );

        waitForStart();

        if (isStopRequested()) return;

        CommandScheduler.getInstance().schedule(autoSequence);

        while (opModeIsActive()) {
            follower.update();
            CommandScheduler.getInstance().run();

            robot.getInstanceLimelight().loop();

            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}
