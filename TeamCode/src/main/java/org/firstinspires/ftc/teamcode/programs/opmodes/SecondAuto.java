package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

/**
 * Example autonomous OpMode demonstrating CustomPathing usage.
 *
 * This shows:
 * - Proper localizer initialization
 * - Trajectory building with different heading modes
 * - Sequential trajectory execution with state machine
 * - Telemetry for debugging
 * - Clean error handling
 */
@Autonomous(name = "CustomPathing Example", group = "Examples")
public class SecondAuto extends LinearOpMode {

    // ==================== CONFIGURATION ====================

    // Adjust these based on your robot's capabilities
    private static final double MAX_VEL = 50.0;        // in/s
    private static final double MAX_ACCEL = 50.0;      // in/s²
    private static final double MAX_DECEL = 60.0;      // in/s²
    private static final double MAX_JERK = 200.0;      // in/s³
    private static final double MAX_CENTRIPETAL = 60.0; // in/s²

    // Path sampling resolution (smaller = smoother but more computation)
    private static final double DS = 0.5;  // inches

    // ==================== HARDWARE ====================

    private MecanumDrive drive;
    private GoBildaPinpointLocalizer localizer;
    private TrajectoryFollower follower;

    // ==================== TRAJECTORIES ====================

    private Trajectory[] trajectories;
    private int currentTrajectoryIndex = 0;

    // ==================== MAIN ====================

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Build all trajectories during init
        buildTrajectories();

        // Display ready status
        telemetry.addLine("=== CustomPathing Ready ===");
        telemetry.addData("Trajectories", trajectories.length);
        telemetry.addData("Total Duration", "%.2f sec", getTotalDuration());
        telemetry.update();

        // Wait for start
        waitForStart();
        if (isStopRequested()) return;

        // Set starting pose to match first trajectory
        if (trajectories.length > 0 && trajectories[0] != null) {
            Trajectory.State startState = trajectories[0].start();
            localizer.setPose(startState.pose);
        }

        // Start first trajectory
        double now = getRuntime();
        startTrajectory(0, now);

        // Main loop
        while (opModeIsActive()) {
            now = getRuntime();

            // Update localizer
            localizer.update();

            // Update follower
            follower.update(now);

            // Check for trajectory completion
            if (follower.isFinished(now)) {
                // Move to next trajectory
                currentTrajectoryIndex++;

                if (currentTrajectoryIndex < trajectories.length) {
                    startTrajectory(currentTrajectoryIndex, now);
                } else {
                    // All trajectories complete
                    follower.cancel();
                    drive.stop();
                    break;
                }
            }

            // Update telemetry
            updateTelemetry(now);

            // Don't hog CPU
            idle();
        }

        // Final telemetry
        telemetry.addLine("=== Autonomous Complete ===");
        Pose2d finalPose = localizer.getPose();
        telemetry.addData("Final Pose", "(%.1f, %.1f) @ %.1f°",
                finalPose.x, finalPose.y, Math.toDegrees(finalPose.heading));
        telemetry.update();
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap);

        // Initialize localizer with your specific configuration
        localizer = new GoBildaPinpointLocalizer("pinpoint")
                .setPodOffsetsMM(-120, 40)  // Adjust for your robot!
                .setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                )
                .setEncoderResolutionPreset(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        localizer.init(hardwareMap);
        localizer.resetPosAndIMU();

        // Initialize follower
        follower = new TrajectoryFollower(localizer::getPose, drive);

        // Configure follower (optional tuning)
        follower.setVelocityFeedforward(1.0);
        follower.setAlphaFeedforward(0.05);
        follower.setCrossTrackToHeadingGain(0.0);
        follower.setPurePursuitThreshold(6.0);
        follower.setRecoverySpeed(30.0);

        telemetry.addLine("Hardware initialized");
        telemetry.update();
    }

    // ==================== TRAJECTORY BUILDING ====================

    private void buildTrajectories() {
        // Create constraints
        TrajectoryConstraints constraints = new TrajectoryConstraints(
                MAX_VEL, MAX_ACCEL, MAX_DECEL, MAX_JERK, MAX_CENTRIPETAL
        );

        // Define waypoints
        Vector2d start = new Vector2d(0, 0);
        Vector2d point1 = new Vector2d(24, 0);
        Vector2d point2 = new Vector2d(24, -24);
        Vector2d point3 = new Vector2d(0, -24);
        Vector2d point4 = new Vector2d(0, 0);

        // Build trajectory sequence
        trajectories = new Trajectory[5];

        // Trajectory 1: Forward to point1 (tangent heading)
        trajectories[0] = new TrajectoryBuilder()
                .line(start, point1)
                .buildTangentHeading(constraints, DS, Math.toRadians(0));

        // Get ending heading for continuity
        double heading1 = getEndHeading(trajectories[0]);

        // Trajectory 2: Turn in place 90° CCW
        Pose2d poseAtPoint1 = new Pose2d(point1.x, point1.y, heading1);
        trajectories[1] = TurnInPlace.buildRelative(poseAtPoint1, Math.toRadians(90), constraints);

        // Trajectory 3: Strafe to point2 (maintain heading)
        double heading2 = getEndHeading(trajectories[1]);
        trajectories[2] = new TrajectoryBuilder()
                .line(point1, point2)
                .buildFixedHeading(constraints, heading2);

        // Trajectory 4: Drive to point3 with heading interpolation
        trajectories[3] = new TrajectoryBuilder()
                .line(point2, point3)
                .buildLinearHeading(constraints, heading2, Math.toRadians(180));

        // Trajectory 5: Return to start
        trajectories[4] = new TrajectoryBuilder()
                .line(point3, point4)
                .buildTangentHeading(constraints, DS, Math.toRadians(180));

        telemetry.addLine("Trajectories built");
    }

    // ==================== TRAJECTORY EXECUTION ====================

    private void startTrajectory(int index, double now) {
        if (index >= 0 && index < trajectories.length && trajectories[index] != null) {
            follower.setTrajectory(trajectories[index], now, false);
            telemetry.addData("Starting trajectory", index + 1);
        }
    }

    // ==================== UTILITY ====================

    private double getEndHeading(Trajectory traj) {
        if (traj == null || traj.allStates().isEmpty()) {
            return 0;
        }
        return traj.end().pose.heading;
    }

    private double getTotalDuration() {
        double total = 0;
        for (Trajectory t : trajectories) {
            if (t != null) {
                total += t.duration();
            }
        }
        return total;
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry(double now) {
        Pose2d pose = localizer.getPose();

        telemetry.addLine("=== CustomPathing ===");
        telemetry.addData("Trajectory", "%d / %d", currentTrajectoryIndex + 1, trajectories.length);
        telemetry.addData("State", follower.getStateString());

        telemetry.addLine("--- Pose ---");
        telemetry.addData("Position", "(%.1f, %.1f) in", pose.x, pose.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading));

        telemetry.addLine("--- Errors ---");
        telemetry.addData("Long Error", "%.2f in", follower.getLongError());
        telemetry.addData("Lat Error", "%.2f in", follower.getLatError());
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(follower.getHeadingError()));

        telemetry.addLine("--- Commands ---");
        telemetry.addData("Vx", "%.1f in/s", follower.getVxCmd());
        telemetry.addData("Vy", "%.1f in/s", follower.getVyCmd());
        telemetry.addData("Omega", "%.1f °/s", Math.toDegrees(follower.getOmegaCmd()));

        telemetry.addLine("--- Motors ---");
        double[] powers = drive.getPowers();
        telemetry.addData("LF/RF", "%.2f / %.2f", powers[0], powers[1]);
        telemetry.addData("LB/RB", "%.2f / %.2f", powers[2], powers[3]);

        if (MecanumDrive.battery != null) {
            telemetry.addData("Battery", "%.2fV", MecanumDrive.battery.getVoltage());
        }

        if (follower.wasInRecovery()) {
            telemetry.addLine("⚠️ Recovery mode was used");
        }

        telemetry.update();
    }
}