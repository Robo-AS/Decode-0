package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Config
@Autonomous(name="TF Calib: Strafe Profile", group="CALIB/TF")
public class TF_StrafeProfileTune extends LinearOpMode {
    public static double START_X = 0, START_Y = 0, START_H_DEG = 0;
    public static double STRAFE_DIST = 36; // +right, -left
    public static double MAX_VEL = 36, MAX_ACC = 48, MAX_JERK = 300, MAX_DECEL = 48, MAX_CENTRIPETAL = 50, DS = 0.5;

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        loc.init(hardwareMap);

        Pose2d start = new Pose2d(START_X, START_Y, Math.toRadians(START_H_DEG));
        Pose2d goal  = new Pose2d(START_X, START_Y + STRAFE_DIST, start.heading);

        loc.setPose(start); loc.resetPosAndIMU();
        TrajectoryConstraints lim = new TrajectoryConstraints(MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);

        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(start.x, start.y), new Vector2d(goal.x, goal.y))
                .buildTangentHeading(lim, DS, null);


        TrajectoryFollower follower = new TrajectoryFollower(loc::getPose, drive);

        waitForStart();
        double t0 = getRuntime(), last = t0;
        follower.setTrajectory(traj, t0, true);

        while (opModeIsActive() && !follower.isFinished(getRuntime())) {
            double now = getRuntime(), dt = Math.max(1e-3, now - last); last = now;
            loc.update(dt); follower.update(now);
            Pose2d p = loc.getPose();
            telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", p.x, p.y, Math.toDegrees(p.heading));
            telemetry.update();
        }
        drive.setPowers(0,0,0,0);
    }
}