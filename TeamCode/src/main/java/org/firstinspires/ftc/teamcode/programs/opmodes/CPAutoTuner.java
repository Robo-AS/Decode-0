package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

/**
 * TUNING OPMODE FOR CUSTOMPATHING
 *
 * Use this OpMode to:
 * 1. Verify hardware configuration
 * 2. Test localizer accuracy
 * 3. Tune PID gains
 * 4. Verify trajectory following
 *
 * Controls (Gamepad 1):
 * - A: Run straight line test (48 inches forward)
 * - B: Run strafe test (48 inches left)
 * - X: Run turn test (90° CCW)
 * - Y: Run square path test
 * - DPAD_UP: Increase selected gain
 * - DPAD_DOWN: Decrease selected gain
 * - DPAD_LEFT/RIGHT: Select gain to tune
 * - LEFT_BUMPER: Reset pose to origin
 * - RIGHT_BUMPER: Emergency stop
 */
@Autonomous(name = "CustomPathing Tuner", group = "Tuning")
public class CPAutoTuner extends LinearOpMode {

    // ==================== HARDWARE ====================
    private MecanumDrive drive;
    private GoBildaPinpointLocalizer localizer;
    private TrajectoryFollower follower;

    // ==================== TUNING STATE ====================
    private enum TuningMode {
        IDLE,
        RUNNING_TEST,
        MANUAL_DRIVE
    }

    private enum GainType {
        KP_X, KI_X, KD_X,
        KP_Y, KI_Y, KD_Y,
        KP_H, KI_H, KD_H,
        MAX_VEL, MAX_ACCEL
    }

    private TuningMode mode = TuningMode.IDLE;
    private GainType selectedGain = GainType.KP_X;
    private String lastTestName = "None";
    private double lastTestError = 0;

    // Tunable values (start with DriveConstants defaults)
    private double kpX = DriveConstants.KP_X;
    private double kiX = DriveConstants.KI_X;
    private double kdX = DriveConstants.KD_X;
    private double kpY = DriveConstants.KP_Y;
    private double kiY = DriveConstants.KI_Y;
    private double kdY = DriveConstants.KD_Y;
    private double kpH = DriveConstants.KP_H;
    private double kiH = DriveConstants.KI_H;
    private double kdH = DriveConstants.KD_H;
    private double maxVel = DriveConstants.MAX_VEL_IN_S;
    private double maxAccel = DriveConstants.MAX_ACCEL_IN_S2;

    // Button debouncing
    private ElapsedTime buttonTimer = new ElapsedTime();
    private static final double BUTTON_DEBOUNCE = 0.2;

    // ==================== MAIN ====================
    @Override
    public void runOpMode() {
        // Initialize
        initHardware();

        telemetry.addLine("=== CustomPathing Tuner Ready ===");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Reset pose
        localizer.resetPosAndIMU();
        sleep(100);
        localizer.setPose(0, 0, 0);

        // Main loop
        while (opModeIsActive()) {
            // Update localizer
            localizer.update();

            // Handle input
            handleInput();

            // Update follower if running
            if (mode == TuningMode.RUNNING_TEST && follower.traj != null) {
                follower.update(getRuntime());

                if (follower.isFinished(getRuntime())) {
                    mode = TuningMode.IDLE;
                    calculateTestError();
                    drive.stop();
                }
            }

            // Display telemetry
            displayTelemetry();

            idle();
        }
    }

    // ==================== INITIALIZATION ====================
    private void initHardware() {
        drive = new MecanumDrive(hardwareMap);

        localizer = new GoBildaPinpointLocalizer("pinpoint")
                .setPodOffsetsMM(-120, 40)  // ADJUST FOR YOUR ROBOT!
                .setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                );
        localizer.init(hardwareMap);

        follower = new TrajectoryFollower(localizer::getPose, drive);
        applyGains();
    }

    // ==================== INPUT HANDLING ====================
    private void handleInput() {
        // Emergency stop
        if (gamepad1.right_bumper) {
            follower.cancel();
            drive.stop();
            mode = TuningMode.IDLE;
            return;
        }

        // Reset pose
        if (gamepad1.left_bumper && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
            localizer.setPose(0, 0, 0);
            buttonTimer.reset();
        }

        // Only allow new tests when idle
        if (mode == TuningMode.IDLE) {
            // Test selection
            if (gamepad1.a && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                runStraightTest();
                buttonTimer.reset();
            } else if (gamepad1.b && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                runStrafeTest();
                buttonTimer.reset();
            } else if (gamepad1.x && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                runTurnTest();
                buttonTimer.reset();
            } else if (gamepad1.y && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                runSquareTest();
                buttonTimer.reset();
            }

            // Gain selection
            if (gamepad1.dpad_left && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                selectPreviousGain();
                buttonTimer.reset();
            } else if (gamepad1.dpad_right && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                selectNextGain();
                buttonTimer.reset();
            }

            // Gain adjustment
            if (gamepad1.dpad_up && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                adjustGain(true);
                buttonTimer.reset();
            } else if (gamepad1.dpad_down && buttonTimer.seconds() > BUTTON_DEBOUNCE) {
                adjustGain(false);
                buttonTimer.reset();
            }
        }
    }

    // ==================== TESTS ====================
    private TrajectoryConstraints getConstraints() {
        return new TrajectoryConstraints(maxVel, maxAccel, maxAccel * 1.2, 200, 60);
    }

    private void runStraightTest() {
        lastTestName = "Straight (48in forward)";
        localizer.setPose(0, 0, 0);
        sleep(50);

        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(0, 0), new Vector2d(48, 0))
                .buildTangentHeading(getConstraints(), 0.5, 0.0);

        follower.setTrajectory(traj, getRuntime(), false);
        mode = TuningMode.RUNNING_TEST;
    }

    private void runStrafeTest() {
        lastTestName = "Strafe (48in left)";
        localizer.setPose(0, 0, 0);
        sleep(50);

        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(0, 0), new Vector2d(0, 48))
                .buildFixedHeading(getConstraints(), Math.toRadians(0.0));

        follower.setTrajectory(traj, getRuntime(), false);
        mode = TuningMode.RUNNING_TEST;
    }

    private void runTurnTest() {
        lastTestName = "Turn (90° CCW)";
        localizer.setPose(0, 0, 0);
        sleep(50);
        TrajectoryConstraints slowTurnConstraints = new TrajectoryConstraints(
                50, 50, 60, 200, 60  // Normal translation constraints
        );
        slowTurnConstraints.maxAngVel = 1.5;      // Very slow: ~86°/sec
        slowTurnConstraints.maxAngAccel = 1.5;

        Trajectory traj = TurnInPlace.buildRelative(
                new Pose2d(0, 0, 0),
                Math.toRadians(90),
                slowTurnConstraints
        );

        follower.setTrajectory(traj, getRuntime(), false);
        mode = TuningMode.RUNNING_TEST;
    }

    private void runSquareTest() {
        lastTestName = "Square (24in sides)";
        localizer.setPose(0, 0, 0);
        sleep(50);

        // Just do first leg for now
        Trajectory traj = new TrajectoryBuilder()
                .line(new Vector2d(0, 0), new Vector2d(24, 0))
                .buildTangentHeading(getConstraints());

        follower.setTrajectory(traj, getRuntime(), false);
        mode = TuningMode.RUNNING_TEST;
    }

    private void calculateTestError() {
        Pose2d current = localizer.getPose();
        Pose2d target;

        switch (lastTestName) {
            case "Straight (48in forward)":
                target = new Pose2d(48, 0, 0);
                break;
            case "Strafe (48in left)":
                target = new Pose2d(0, 48, Math.toRadians(90));
                break;
            case "Turn (90° CCW)":
                target = new Pose2d(0, 0, Math.toRadians(90));
                break;
            case "Square (24in sides)":
                target = new Pose2d(24, 0, 0);
                break;
            default:
                target = new Pose2d(0, 0, 0);
        }

        double posError = Math.hypot(current.x - target.x, current.y - target.y);
        double headError = Math.abs(HeadingUtil.shortestDelta(current.heading, target.heading));
        lastTestError = posError + Math.toDegrees(headError) * 0.1; // Combined metric
    }

    // ==================== GAIN MANAGEMENT ====================
    private void selectNextGain() {
        GainType[] values = GainType.values();
        int idx = (selectedGain.ordinal() + 1) % values.length;
        selectedGain = values[idx];
    }

    private void selectPreviousGain() {
        GainType[] values = GainType.values();
        int idx = (selectedGain.ordinal() - 1 + values.length) % values.length;
        selectedGain = values[idx];
    }

    private void adjustGain(boolean increase) {
        double factor = increase ? 1.1 : 0.9;
        double delta = increase ? 0.001 : -0.001;

        switch (selectedGain) {
            case KP_X: kpX = Math.max(0.1, kpX * factor); break;
            case KI_X: kiX = Math.max(0, kiX + delta); break;
            case KD_X: kdX = Math.max(0, kdX * factor); break;
            case KP_Y: kpY = Math.max(0.1, kpY * factor); break;
            case KI_Y: kiY = Math.max(0, kiY + delta); break;
            case KD_Y: kdY = Math.max(0, kdY * factor); break;
            case KP_H: kpH = Math.max(0.1, kpH * factor); break;
            case KI_H: kiH = Math.max(0, kiH + delta * 0.1); break;
            case KD_H: kdH = Math.max(0, kdH * factor); break;
            case MAX_VEL: maxVel = Math.max(10, maxVel + (increase ? 5 : -5)); break;
            case MAX_ACCEL: maxAccel = Math.max(10, maxAccel + (increase ? 5 : -5)); break;
        }

        applyGains();
    }

    private void applyGains() {
        follower.setGains(kpX, kiX, kdX, kpY, kiY, kdY, kpH, kiH, kdH);
    }

    private double getSelectedGainValue() {
        switch (selectedGain) {
            case KP_X: return kpX;
            case KI_X: return kiX;
            case KD_X: return kdX;
            case KP_Y: return kpY;
            case KI_Y: return kiY;
            case KD_Y: return kdY;
            case KP_H: return kpH;
            case KI_H: return kiH;
            case KD_H: return kdH;
            case MAX_VEL: return maxVel;
            case MAX_ACCEL: return maxAccel;
            default: return 0;
        }
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        Pose2d pose = localizer.getPose();

        // Add these during the turn (in the main loop, showing continuously)
        telemetry.addData("State", follower.getStateString());
        telemetry.addData("Vx Cmd", "%.3f", follower.getVxCmd());
        telemetry.addData("Vy Cmd", "%.3f", follower.getVyCmd());
        telemetry.addData("Omega Cmd", "%.3f", follower.getOmegaCmd());
        telemetry.addData("Long Err", "%.3f", follower.getLongError());
        telemetry.addData("Lat Err", "%.3f", follower.getLatError());
        telemetry.addData("Head Err", "%.3f", Math.toDegrees(follower.getHeadingError()));

// Also add current pose
        telemetry.addData("X", "%.2f", pose.x);
        telemetry.addData("Y", "%.2f", pose.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(pose.heading));

        telemetry.addData("Vx Cmd", "%.2f", follower.getVxCmd());
        telemetry.addData("Vy Cmd", "%.2f", follower.getVyCmd());
        telemetry.addData("Omega Cmd", "%.2f", follower.getOmegaCmd());
        telemetry.addData("Long Error", "%.2f", follower.getLongError());
        telemetry.addData("Lat Error", "%.2f", follower.getLatError());
        telemetry.addData("State", follower.getStateString());

        telemetry.addLine("════════ CUSTOMPATHING TUNER ════════");
        telemetry.addData("Mode", mode);
        telemetry.addData("Pinpoint Status", localizer.getStatusString());

        telemetry.addLine("──────── POSE ────────");
        telemetry.addData("X", "%.2f in", pose.x);
        telemetry.addData("Y", "%.2f in", pose.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading));

        telemetry.addLine("──────── TESTS (press when IDLE) ────────");
        telemetry.addData("A", "Straight 48in");
        telemetry.addData("B", "Strafe 48in");
        telemetry.addData("X", "Turn 90°");
        telemetry.addData("Y", "Square path");

        telemetry.addLine("──────── LAST TEST ────────");
        telemetry.addData("Test", lastTestName);
        telemetry.addData("Error", "%.2f", lastTestError);

        telemetry.addLine("──────── TUNING (DPAD) ────────");
        telemetry.addData("Selected", ">>> %s <<<", selectedGain);
        telemetry.addData("Value", "%.4f", getSelectedGainValue());
        telemetry.addLine("DPAD L/R: Select | DPAD U/D: Adjust");

        telemetry.addLine("──────── ALL GAINS ────────");
        telemetry.addData("X PID", "P=%.2f I=%.4f D=%.3f", kpX, kiX, kdX);
        telemetry.addData("Y PID", "P=%.2f I=%.4f D=%.3f", kpY, kiY, kdY);
        telemetry.addData("H PID", "P=%.2f I=%.4f D=%.3f", kpH, kiH, kdH);
        telemetry.addData("Limits", "V=%.0f A=%.0f", maxVel, maxAccel);

        telemetry.addLine("──────── CONTROLS ────────");
        telemetry.addData("LB", "Reset pose | RB: Emergency stop");

        if (mode == TuningMode.RUNNING_TEST) {
            telemetry.addLine("──────── LIVE ERRORS ────────");
            telemetry.addData("Long Error", "%.2f in", follower.getLongError());
            telemetry.addData("Lat Error", "%.2f in", follower.getLatError());
            telemetry.addData("Head Error", "%.1f°", Math.toDegrees(follower.getHeadingError()));
            telemetry.addData("State", follower.getStateString());
        }

        // Print final gain values for copy-paste
        telemetry.addLine("──────── COPY TO DriveConstants.java ────────");
        telemetry.addLine(String.format("KP_X=%.2f; KI_X=%.4f; KD_X=%.3f;", kpX, kiX, kdX));
        telemetry.addLine(String.format("KP_Y=%.2f; KI_Y=%.4f; KD_Y=%.3f;", kpY, kiY, kdY));
        telemetry.addLine(String.format("KP_H=%.2f; KI_H=%.4f; KD_H=%.3f;", kpH, kiH, kdH));

        telemetry.update();
    }
}