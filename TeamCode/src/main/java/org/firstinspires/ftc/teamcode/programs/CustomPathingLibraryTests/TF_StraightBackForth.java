package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

@Autonomous(name = "TF_ABA_Spins_Raw", group = "Tuning")
public class TF_StraightBackForth extends LinearOpMode {
    private final CompositePath comppath = new CompositePath();

    public static double MAX_VEL = 36, MAX_ACC = 48, MAX_JERK = 300, MAX_DECEL = 48, MAX_CENTRIPETAL = 50, DS = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        GoBildaPinpointLocalizer localizer = new GoBildaPinpointLocalizer("pinpoint").setPodOffsetsMM(-84,-168);
        localizer.init(hardwareMap);
        localizer.setPose(new Pose2d(0,0,0));

        TrajectoryFollower follower = new TrajectoryFollower(localizer::getPose, drive);
        TrajectoryConstraints lim = new TrajectoryConstraints(MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL);

        Vector2d A = new Vector2d(0,0);
        Vector2d B = new Vector2d(60,0);

        Trajectory t1 = new TrajectoryBuilder()
                .line(A,B)
                .buildTangentHeading(lim,0.5);

        Trajectory t2 = new TrajectoryBuilder()
                .line(B,new Vector2d(B.x+1,B.y))
                .buildWithHeading(lim,0.5,s->{
                    Vector2d p = comppath.pointAtS(s);
                    double dx = A.x - p.x, dy = A.y - p.y;
                    return Math.atan2(dy,dx);
                });

        Trajectory t3 = new TrajectoryBuilder()
                .line(B,A)
                .buildTangentHeading(lim,0.5);

        Trajectory t4 = new TrajectoryBuilder()
                .line(A,new Vector2d(A.x+1,A.y))
                .buildWithHeading(lim,0.5,s->{
                    Vector2d p = comppath.pointAtS(s);
                    double dx = B.x - p.x, dy = B.y - p.y;
                    return Math.atan2(dy,dx);
                });

        Trajectory[] seq = new Trajectory[]{t1,t2,t3,t4};

        waitForStart();
        if (isStopRequested()) return;

        int idx = 0;
        double now = getRuntime();
        follower.setTrajectory(seq[idx], now);

        while (opModeIsActive()) {
            now = getRuntime();
            follower.update(now);
            if (follower.isFinished(now)) {
                idx = (idx+1)%seq.length;
                follower.setTrajectory(seq[idx], now);
            }
            idle();
        }
    }
}