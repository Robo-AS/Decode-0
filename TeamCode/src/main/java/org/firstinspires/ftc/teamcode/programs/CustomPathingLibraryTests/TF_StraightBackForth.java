package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

@Autonomous(name = "TF_ABA_Sequence_Fixed", group = "Tuning")
public class TF_StraightBackForth extends LinearOpMode {

    // Use DriveConstants in real code; kept local for a quick test
    public static double MAX_VEL = 60, MAX_ACC = 40, MAX_JERK = 200, MAX_DECEL = 20, MAX_CENTRIPETAL = 80, DS = 0.5;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        GoBildaPinpointLocalizer localizer =
                new GoBildaPinpointLocalizer("pinpoint")
                        .setPodOffsetsMM(-100, 160) // X left+, Y forward+
                        .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        localizer.init(hardwareMap);
        localizer.resetPosAndIMU(); // still robot → gyro bias set

        TrajectoryFollower follower = new TrajectoryFollower(localizer::getPose, drive);
        follower.setAlphaFeedforward(0.08);
        follower.setCrossTrackToHeadingGain(0.2);

        TrajectoryConstraints lim = new TrajectoryConstraints(
                MAX_VEL, MAX_ACC, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL
        );

        Vector2d A = new Vector2d(0, 0);
        Vector2d B = new Vector2d(30, 0);

        // A -> B, tangent heading
        Trajectory t1 = new TrajectoryBuilder()
                .line(A, B)
                .buildTangentHeading(lim, DS, null);

        // Store its end heading for continuity
        double seed12 = t1.allStates().get(t1.allStates().size()-1).pose.heading;


        // In-place 180 face back toward A using a fixed heading profile across a 1-inch "stub"
        double hA = Math.atan2(0, 1);            // 0 rad
        double hB = hA - (Math.PI/4);                // +π rad (180°)
        Trajectory t2 = new TrajectoryBuilder()
                .line(B, new Vector2d(B.x + 12, B.y)) // tiny segment just to carry heading profile
                .buildWithHeading(lim, DS, new FixedStartEndHeading(hA, hB, 1.0), seed12);

        double seed23 = t2.allStates().get(t2.allStates().size()-1).pose.heading;


        // B -> A, tangent heading (which is already pointing back)
        Trajectory t3 = new TrajectoryBuilder()
                .line(B, A)
                .buildTangentHeading(lim, DS, null);

        // Turn back to original heading on a 1-inch stub at A
        Trajectory t4 = new TrajectoryBuilder()
                .line(A, new Vector2d(A.x + 12, A.y))
                .buildWithHeading(lim, DS, new FixedStartEndHeading(hB, hA, 1.0), null);

        Trajectory[] seq = new Trajectory[]{t1, t2, t3, t4};
        int idx = 0;

        // Sync start pose to the first trajectory
        Trajectory.State s0 = t1.sample(0);
        localizer.setPose(s0.pose);

        waitForStart();
        if (isStopRequested()) return;

        double now = getRuntime();
        double lastNow = now;
        int stage = 1;
        follower.setTrajectory(t1, now);


        while (opModeIsActive()) {
            now = getRuntime();
            double dt = now-lastNow;
            lastNow=now;

            // Update the localizer FIRST
            localizer.update(dt); // (dt arg unused in localizer code)

            // Then the follower
            follower.update(now);

            if (stage == 1 && follower.isFinished(now)) {
                follower.setTrajectory(t3, now+100);
                stage = 2;
            }
//            else if (stage == 2 && follower.isFinished(now)) {
//                follower.setTrajectory(t3, now);
//                stage = 3;
//            }
//            else if (stage == 3 && follower.isFinished(now)) {
//                follower.setTrajectory(t4, now);
//                stage = 4;
//            }
            else if (stage == 2 && follower.isFinished(now)) {
                // Done
                drive.setPowers(0,0,0,0);
                break;
            }

            // telemetry
            telemetry.addData("Stage", stage);


            Pose2d p = localizer.getPose();
            telemetry.addData("pose","x=%.1f y=%.1f h=%.0f°", p.x, p.y, Math.toDegrees(p.heading));
            telemetry.addData("LF", "%.2f", drive.LeftFront.getPower());
            telemetry.addData("RF", "%.2f", drive.RightFront.getPower());
            telemetry.addData("LB", "%.2f", drive.LeftRear.getPower());
            telemetry.addData("RB", "%.2f", drive.RightRear.getPower());
            double tEl = now - /* the same t0 you pass inside follower, or: */ 0; // easier: expose follower.getElapsed()
            Trajectory.State ref = t1.sample(Math.min(tEl, t1.duration())); // for first leg, or use follower’s current ref
            telemetry.addData("traj", "elapsed=%.2f dur=%.2f ref.s=%.2f end.s=%.2f",
                    now, follower.traj.duration(), ref.s, follower.endState.s);
            for (int i = 0; i < Math.min(5, t1.allStates().size()); i++) {
                Trajectory.State s = t1.allStates().get(i);
                telemetry.addData("t1[%d]", "s=%.2f v=%.2f a=%.2f", s.s, s.v, s.a);
            }
            telemetry.update();
            telemetry.update();

            idle();
        }
    }
}