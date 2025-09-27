package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;
import java.util.*;

@Config
@Autonomous(name="Localization: Field Coordinate Check", group="CALIB/LOC")
public class Localization_FieldCoordinateCheck extends LinearOpMode {
    public static double X0=0, Y0=0, X1=48, Y1=0;
    public static double MAX_VEL = 36, MAX_ACC = 48, MAX_JERK = 300, MAX_DECEL = 48, MAX_CENTRIPETAL = 50, DS = 0.5;

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        loc.init(hardwareMap);
        Pose2d start = new Pose2d(X0,Y0,0); loc.setPose(start); loc.resetPosAndIMU();

        TrajectoryConstraints lim = new TrajectoryConstraints(MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);

        Trajectory out = new TrajectoryBuilder()
                .line(new Vector2d(X0,Y0), new Vector2d(X1,Y1))
                .buildTangentHeading(lim, DS);

        Trajectory back = new TrajectoryBuilder()
                .line(new Vector2d(X1,Y1), new Vector2d(X0,Y0))
                .buildTangentHeading(lim, DS);

        TrajectoryFollower follower = new TrajectoryFollower(loc::getPose, drive);

        waitForStart();
        double last = getRuntime();

        follower.setTrajectory(out, getRuntime());
        while(opModeIsActive() && !follower.isFinished(getRuntime())){
            double now=getRuntime(), dt=Math.max(1e-3, now-last); last=now;
            loc.update(dt); follower.update(now);
            telemetry.addData("phase","out");
            Pose2d p=loc.getPose(); telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", p.x,p.y,Math.toDegrees(p.heading));
            telemetry.update();
        }
        follower.setTrajectory(back, getRuntime());
        while(opModeIsActive() && !follower.isFinished(getRuntime())){
            double now=getRuntime(), dt=Math.max(1e-3, now-last); last=now;
            loc.update(dt); follower.update(now);
            telemetry.addData("phase","back");
            Pose2d p=loc.getPose(); telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", p.x,p.y,Math.toDegrees(p.heading));
            telemetry.update();
        }

        Pose2d end = loc.getPose();
        telemetry.addLine("--- Result ---");
        telemetry.addData("Start","(%.1f, %.1f)", X0, Y0);
        telemetry.addData("End  ","(%.1f, %.1f)", end.x, end.y);
        telemetry.addData("Error","dx=%.2f dy=%.2f", end.x-X0, end.y-Y0);
        telemetry.update();
        drive.setPowers(0,0,0,0);
    }
}