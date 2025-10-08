package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Config
@Autonomous(name="TF Calib: Straight Profile (Matches Builder)", group="CALIB")
public class TF_StraightProfileTune extends LinearOpMode {

    public static double START_X = 0, START_Y = 0, START_H_DEG = 0;
    public static double GOAL_X  = 48, GOAL_Y  = 0, GOAL_H_DEG  = 0;

    // motion limits for time-parameterizer
    public static double MAX_VEL = 36;     // in/s
    public static double MAX_ACC = 48;     // in/s^2
    public static double MAX_JERK = 300;   // in/s^3
    public static double MAX_DECEL = 48;
    public static double MAX_CENTRIPETAL = 50;
    public static double DS = 0.5;         // path sample step (in)

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint")
                .setPodOffsetsMM(-84, -168);
        loc.init(hardwareMap);

        Pose2d start = new Pose2d(START_X, START_Y, Math.toRadians(START_H_DEG));
        Pose2d goal  = new Pose2d(GOAL_X,  GOAL_Y,  Math.toRadians(GOAL_H_DEG));

        loc.setPose(start);
        loc.resetPosAndIMU();

        TrajectoryConstraints lim = new TrajectoryConstraints(MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);

        TrajectoryBuilder tb = new TrajectoryBuilder()
                .line(new Vector2d(start.x, start.y), new Vector2d(goal.x, goal.y));


         Trajectory traj = tb.buildWithHeading(lim, DS, new FixedStartEndHeading(start.heading, goal.heading, 10), null);

        TrajectoryFollower follower = new TrajectoryFollower(loc::getPose, drive);

        waitForStart();
        double t0 = getRuntime(), last = t0;
        follower.setTrajectory(traj, t0);

        while (opModeIsActive() && !follower.isFinished(getRuntime())) {
            double now = getRuntime(), dt = Math.max(1e-3, now - last); last = now;
            loc.update(dt);
            follower.update(now);

            Pose2d p = loc.getPose();
            telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", p.x, p.y, Math.toDegrees(p.heading));
            telemetry.addData("limits","V=%.0f A=%.0f J=%.0f  ds=%.1f", MAX_VEL, MAX_ACC, MAX_JERK, DS);
            telemetry.update();
        }
        drive.setPowers(0,0,0,0);
    }
}