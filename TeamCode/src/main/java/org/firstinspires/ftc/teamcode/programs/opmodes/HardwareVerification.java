package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.*;

import Pinpoint_Blocks_Driver.GoBildaPinpointDriver;

/**
 * HARDWARE VERIFICATION TEST
 *
 * Run this FIRST before any tuning to verify:
 * 1. All motors are connected and named correctly
 * 2. Motor directions are correct
 * 3. Pinpoint is connected and working
 * 4. Encoder directions are correct
 *
 * Controls:
 * - Left Stick: Drive robot manually (robot-centric)
 * - Right Stick X: Rotate
 * - A: Test Left Front motor only
 * - B: Test Right Front motor only
 * - X: Test Left Rear motor only
 * - Y: Test Right Rear motor only
 * - DPAD_UP: All motors forward
 * - DPAD_DOWN: All motors backward
 * - LB: Reset Pinpoint pose
 * - RB: Recalibrate IMU
 */
@TeleOp(name = "Hardware Verification", group = "Tuning")
public class HardwareVerification extends LinearOpMode {

    private MecanumDrive drive;
    private GoBildaPinpointLocalizer localizer;

    private boolean motorsOk = false;
    private boolean pinpointOk = false;
    private String errorMessage = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("=== Hardware Verification Test ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Try to initialize hardware
        initHardware();

        // Display init results
        telemetry.addLine("=== Initialization Results ===");
        telemetry.addData("Motors", motorsOk ? "✓ OK" : "✗ FAILED");
        telemetry.addData("Pinpoint", pinpointOk ? "✓ OK" : "✗ FAILED");
        if (!errorMessage.isEmpty()) {
            telemetry.addLine("Error: " + errorMessage);
        }
        telemetry.addLine("");
        telemetry.addLine("Press START to begin testing");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Reset Pinpoint if available
        if (pinpointOk) {
            localizer.resetPosAndIMU();
            sleep(100);
        }

        // Main test loop
        while (opModeIsActive()) {
            // Handle controls
            handleControls();

            // Update localizer
            if (pinpointOk) {
                localizer.update();
            }

            // Display telemetry
            displayTelemetry();

            idle();
        }

        // Stop motors
        if (motorsOk) {
            drive.stop();
        }
    }

    private void initHardware() {
        // Try motors
        try {
            drive = new MecanumDrive(hardwareMap);
            motorsOk = true;
        } catch (Exception e) {
            motorsOk = false;
            errorMessage = "Motors: " + e.getMessage();
        }

        // Try Pinpoint
        try {
            localizer = new GoBildaPinpointLocalizer("pinpoint")
                    .setPodOffsetsMM(-120, 40)  // ADJUST FOR YOUR ROBOT!
                    .setEncoderDirections(
                            GoBildaPinpointDriver.EncoderDirection.REVERSED,
                            GoBildaPinpointDriver.EncoderDirection.REVERSED
                    );
            localizer.init(hardwareMap);
            pinpointOk = localizer.isInitialized();
            if (!pinpointOk) {
                errorMessage += " Pinpoint: Not initialized";
            }
        } catch (Exception e) {
            pinpointOk = false;
            errorMessage += " Pinpoint: " + e.getMessage();
        }
    }

    private void handleControls() {
        if (!motorsOk) return;

        // Individual motor tests
        if (gamepad1.a) {
            drive.setPowers(0.3, 0, 0, 0);  // LF only
        } else if (gamepad1.b) {
            drive.setPowers(0, 0.3, 0, 0);  // RF only
        } else if (gamepad1.x) {
            drive.setPowers(0, 0, 0.3, 0);  // LB only
        } else if (gamepad1.y) {
            drive.setPowers(0, 0, 0, 0.3);  // RB only
        } else if (gamepad1.dpad_up) {
            drive.setPowers(0.3, 0.3, 0.3, 0.3);  // All forward
        } else if (gamepad1.dpad_down) {
            drive.setPowers(-0.3, -0.3, -0.3, -0.3);  // All backward
        } else {
            // Normal drive with sticks
            double forward = -gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.5;
            double rotate = -gamepad1.right_stick_x * 0.5;

            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(strafe) < 0.05) strafe = 0;
            if (Math.abs(rotate) < 0.05) rotate = 0;

            drive.driveRobotCentric(forward, strafe, rotate);
        }

        // Pinpoint controls
        if (pinpointOk) {
            if (gamepad1.left_bumper) {
                localizer.setPose(0, 0, 0);
            }
            if (gamepad1.right_bumper) {
                localizer.recalibrateIMU();
            }
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("════════ HARDWARE VERIFICATION ════════");

        // Motor status
        telemetry.addLine("──────── MOTORS ────────");
        if (motorsOk) {
            telemetry.addData("Status", "✓ Connected");
            telemetry.addData("LF Power", "%.2f", drive.LeftFront.getPower());
            telemetry.addData("RF Power", "%.2f", drive.RightFront.getPower());
            telemetry.addData("LB Power", "%.2f", drive.LeftRear.getPower());
            telemetry.addData("RB Power", "%.2f", drive.RightRear.getPower());

            telemetry.addLine("");
            telemetry.addData("LF Encoder", drive.LeftFront.getCurrentPosition());
            telemetry.addData("RF Encoder", drive.RightFront.getCurrentPosition());
            telemetry.addData("LB Encoder", drive.LeftRear.getCurrentPosition());
            telemetry.addData("RB Encoder", drive.RightRear.getCurrentPosition());
        } else {
            telemetry.addData("Status", "✗ ERROR - Check configuration!");
        }

        // Battery
        telemetry.addLine("");
        if (MecanumDrive.battery != null) {
            telemetry.addData("Battery", "%.2fV", MecanumDrive.battery.getVoltage());
        } else {
            telemetry.addData("Battery", "No sensor found");
        }

        // Pinpoint status
        telemetry.addLine("──────── PINPOINT ────────");
        if (pinpointOk) {
            Pose2d pose = localizer.getPose();
            telemetry.addData("Status", "✓ " + localizer.getStatusString());
            telemetry.addData("Frequency", "%.0f Hz", localizer.getFrequency());
            telemetry.addData("X", "%.2f in", pose.x);
            telemetry.addData("Y", "%.2f in", pose.y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading));
            telemetry.addLine("");
            telemetry.addData("Vx", "%.1f in/s", localizer.getVX());
            telemetry.addData("Vy", "%.1f in/s", localizer.getVY());
            telemetry.addData("ω", "%.1f °/s", Math.toDegrees(localizer.getOmega()));
        } else {
            telemetry.addData("Status", "✗ NOT CONNECTED");
            telemetry.addLine("Check that 'pinpoint' is configured as");
            telemetry.addLine("GoBildaPinpointDriver in robot config!");
        }

        // Direction verification guide
        telemetry.addLine("──────── DIRECTION TESTS ────────");
        telemetry.addLine("Push robot FORWARD → X should ↑");
        telemetry.addLine("Push robot LEFT → Y should ↑");
        telemetry.addLine("Rotate robot CCW → Heading should ↑");
        telemetry.addLine("");
        telemetry.addLine("If wrong, change encoder directions in code");

        // Controls reference
        telemetry.addLine("──────── CONTROLS ────────");
        telemetry.addLine("A/B/X/Y: Test individual motors");
        telemetry.addLine("DPAD Up/Down: All motors fwd/back");
        telemetry.addLine("Left Stick: Drive | Right Stick: Rotate");
        telemetry.addLine("LB: Reset pose | RB: Recalibrate IMU");

        // Motor direction check
        telemetry.addLine("──────── MOTOR DIRECTION CHECK ────────");
        telemetry.addLine("Press DPAD_UP - robot should go FORWARD");
        telemetry.addLine("If a motor goes wrong way, reverse it:");
        telemetry.addLine("In MecanumDrive.java, change FORWARD↔REVERSE");

        telemetry.update();
    }
}