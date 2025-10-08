package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Config
@Autonomous(name="TF Calib: Velocity Tracking", group="CALIB/TF")
public class TF_VelocityTrackingTune extends LinearOpMode {
    public static double X0=0, Y0=0, X1=84, Y1=0; // long line
    public static double CMD_MAX_VEL=48, MAX_ACC=48, MAX_DECEL = 48, MAX_JERK=300, MAX_CENTRIPETAL = 50, DS=0.5;

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        loc.init(hardwareMap);
        loc.setPose(new Pose2d(X0, Y0, 0)); loc.resetPosAndIMU();

        TrajectoryConstraints lim = new TrajectoryConstraints(CMD_MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);
        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(X0,Y0), new Vector2d(X1,Y1))
                .buildTangentHeading(lim, DS, null);

        TrajectoryFollower follower = new TrajectoryFollower(loc::getPose, drive);

        waitForStart();
        double t0=getRuntime(), last=t0; follower.setTrajectory(traj, t0);
        while (opModeIsActive() && !follower.isFinished(getRuntime())) {
            double now=getRuntime(), dt=Math.max(1e-3, now-last); last=now;
            loc.update(dt); follower.update(now);
            Pose2d p = loc.getPose();
            telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", p.x, p.y, Math.toDegrees(p.heading));
            telemetry.addData("cmdMaxVel","%.1f in/s", CMD_MAX_VEL);
            telemetry.update();
        }
        drive.setPowers(0,0,0,0);
    }
}