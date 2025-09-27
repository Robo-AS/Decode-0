package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Config
@Autonomous(name="Localization: Drift Test", group="CALIB/LOC")
public class Localization_DriftTest extends LinearOpMode {
    public static double VX=10, VY=0, W=0; // in/s & rad/s
    public static double SECONDS=3.0;

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer loc = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        loc.init(hardwareMap);
        loc.setPose(new Pose2d(0,0,0)); loc.resetPosAndIMU();

        waitForStart();
        double t0=getRuntime(), last=t0;
        while (opModeIsActive() && (getRuntime()-t0)<=SECONDS){
            double now=getRuntime(), dt=Math.max(1e-3, now-last); last=now;
            loc.update(dt);

            // feed chassis velocity directly (simple mapper)
            double[] ws = new double[4];
            MecanumKinematics.chassisToWheels(VX, VY, W, DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN, ws);
            double[] p = new double[4];
            MecanumKinematics.normalizeToPowers(ws, DriveConstants.MAX_WHEEL_SPEED_IN_S, p);
            drive.setPowers(p[0], p[1], p[2], p[3]);

            Pose2d pose=loc.getPose();
            telemetry.addData("pose","x=%.1f y=%.1f h=%.1f°", pose.x, pose.y, Math.toDegrees(pose.heading));
            telemetry.addData("cmd","vx=%.1f vy=%.1f w=%.2f", VX, VY, W);
            telemetry.update();
            sleep(20);
        }
        drive.setPowers(0,0,0,0);
    }
}