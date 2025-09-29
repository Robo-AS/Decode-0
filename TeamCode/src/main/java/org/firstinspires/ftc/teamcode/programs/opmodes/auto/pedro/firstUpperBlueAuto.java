package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.commandbase.auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Autonomous(name = "Bottom Blue Auto")
public class firstUpperBlueAuto extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();

    public static Pose startPose = new Pose(56.000, 135.400, Math.toRadians(90));
    public static Pose outtakePreload = new Pose(32.626, 110.690);
    public static Pose outtake1 = new Pose(51.758, 91.730);
    public static Pose outtake2 =  new Pose(64.740, 79.089);

    public static Pose intake1 = new Pose(45.000, 84.500);
    public static Pose intake2 = new Pose(45.000, 59.000);


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        follower = Constants.createFollower(Robot.getInstanceHardwareMap());
        follower.setStartingPose(startPose);

        robot.initializeHardware(hardwareMap, new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        ));
        robot.initialize();

        PathBuilder builder = new PathBuilder(follower, new PathConstraints(30, 30));

        PathChain launchPreload = builder
                .addPath(
                        new BezierLine(startPose, outtakePreload)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        PathChain get1 = builder
                .addPath(
                        new BezierLine(outtakePreload, intake1)
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
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
