package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Config
@Autonomous(name="TF Calib: Turn Profile", group="CALIB/TF")
public class TF_TurnProfileTune extends LinearOpMode {
    public static double X=0, Y=0, H0_DEG=0, H1_DEG=180;
    public static double MAX_VEL=24, MAX_ACC=48, MAX_DECEL = 48, MAX_JERK=300, MAX_CENTRIPETAL = 50, DS=0.5;

    // simple heading profile: linear from h0->h1 along s
    static class FixedStartEndHeading implements HeadingProfile {
        private final double h0, h1;
        FixedStartEndHeading(double h0, double h1){ this.h0=h0; this.h1=h1; }
        @Override
        public double headingAt(double s) { return 0; } // unused by this build
        public double headingAtS(double s, double sTotal) {
            double u = (sTotal <= 1e-6) ? 1.0 : (s / sTotal);
            return h0 + (h1 - h0) * Math.max(0, Math.min(1, u));
        }
    }

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        loc.init(hardwareMap);

        Pose2d start = new Pose2d(X, Y, Math.toRadians(H0_DEG));
        Pose2d end   = new Pose2d(X + 1e-3, Y, Math.toRadians(H1_DEG)); // ~no translation
        loc.setPose(start); loc.resetPosAndIMU();

        TrajectoryConstraints lim = new TrajectoryConstraints(MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);

        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(start.x, start.y), new Vector2d(end.x, end.y))
                .buildWithHeading(lim, DS, (s)->0 , null); // we’ll override heading below

        // Rebuild with a heading profile based on s (use total length)
        double totalS = new CompositePath().length(); // if unavailable, just reuse builder’s internal length
        FixedStartEndHeading prof = new FixedStartEndHeading(start.heading, Math.toRadians(H1_DEG));
        traj = new TrajectoryBuilder()
                .line(new Vector2d(start.x, start.y), new Vector2d(end.x, end.y))
                .buildWithHeading(lim, DS, s -> prof.headingAtS(s, 1.0), null);

        TrajectoryFollower follower = new TrajectoryFollower(loc::getPose, drive);

        waitForStart();
        double t0 = getRuntime(), last = t0;
        follower.setTrajectory(traj, t0);

        while (opModeIsActive() && !follower.isFinished(getRuntime())) {
            double now = getRuntime(), dt = Math.max(1e-3, now - last); last = now;
            loc.update(dt); follower.update(now);
            Pose2d p = loc.getPose();
            telemetry.addData("pose","x=%.2f y=%.2f h=%.1f°", p.x, p.y, Math.toDegrees(p.heading));
            telemetry.update();
        }
        drive.setPowers(0,0,0,0);
    }
}