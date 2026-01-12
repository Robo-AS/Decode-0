package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

/**
 * Mecanum drive hardware abstraction.
 *
 * Handles:
 * - Motor initialization and configuration
 * - TeleOp driving (field-centric and robot-centric)
 * - Autonomous power output
 * - Voltage sensing for compensation
 *
 * Motor order convention: [LF, RF, LB, RB]
 */
public class MecanumDrive {

    // ==================== HARDWARE ====================

    public DcMotorEx LeftFront;
    public DcMotorEx RightFront;
    public DcMotorEx LeftRear;
    public DcMotorEx RightRear;

    private List<DcMotorEx> motors;

    // Voltage sensor (static for access from other classes)
    public static VoltageSensor battery = null;

    // ==================== CONFIGURATION ====================

    // TeleOp settings
    private double speedMultiplier = 1.0;
    private double turnMultiplier = 0.8;
    private double deadzone = 0.05;
    private boolean fieldCentric = false;

    // Heading supplier for field-centric (set externally)
    private java.util.function.DoubleSupplier headingSupplier = () -> 0.0;

    // Direction reversal for driver preference
    private double directionMultiplier = 1.0;

    // ==================== CONSTRUCTOR ====================

    /**
     * Initialize mecanum drive with hardware map.
     * Uses default motor names: "LeftFront", "RightFront", "LeftRear", "RightRear"
     */
    public MecanumDrive(HardwareMap hardwareMap) {
        this(hardwareMap, "LeftFront", "RightFront", "LeftRear", "RightRear");
    }

    /**
     * Initialize mecanum drive with custom motor names.
     */
    public MecanumDrive(HardwareMap hardwareMap,
                        String lfName, String rfName, String lbName, String rbName) {

        // Get motors
        LeftFront = hardwareMap.get(DcMotorEx.class, lfName);
        RightFront = hardwareMap.get(DcMotorEx.class, rfName);
        LeftRear = hardwareMap.get(DcMotorEx.class, lbName);
        RightRear = hardwareMap.get(DcMotorEx.class, rbName);

        motors = Arrays.asList(LeftFront, RightFront, LeftRear, RightRear);

        // Configure motors
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set motor directions (adjust based on your robot's wiring)
        // Standard: left motors reversed, right motors forward
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Find best voltage sensor
        battery = null;
        double bestVoltage = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (Double.isFinite(v) && v > bestVoltage) {
                bestVoltage = v;
                battery = vs;
            }
        }
    }

    // ==================== TELEOP DRIVING ====================

    /**
     * Standard TeleOp driving (robot-centric by default).
     *
     * @param gamepad  FTCLib GamepadEx
     * @param telemetry  Telemetry for debugging
     */
    public void teleop(GamepadEx gamepad, Telemetry telemetry) {
        double leftX = -gamepad.getLeftX();   // Strafe
        double leftY = -gamepad.getLeftY();   // Forward/backward
        double rightX = -gamepad.getRightX(); // Rotation

        // Apply deadzone
        leftX = applyDeadzone(leftX);
        leftY = applyDeadzone(leftY);
        rightX = applyDeadzone(rightX);

        // Apply direction reversal
        leftX *= directionMultiplier;
        leftY *= directionMultiplier;

        // Apply multipliers
        leftX *= speedMultiplier;
        leftY *= speedMultiplier;
        rightX *= speedMultiplier * turnMultiplier;

        // Field-centric conversion if enabled
        if (fieldCentric) {
            double heading = headingSupplier.getAsDouble();
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);

            double temp = leftY * cos - leftX * sin;
            leftX = leftY * sin + leftX * cos;
            leftY = temp;
        }

        // Calculate motor powers
        driveRobotCentric(leftY, leftX, rightX);
    }

    /**
     * Slow mode TeleOp (reduced speed).
     */
    public void Slow_Motion(GamepadEx gamepad, Telemetry telemetry) {
        double slowFactor = 0.3;
        double oldMultiplier = speedMultiplier;
        speedMultiplier *= slowFactor;
        teleop(gamepad, telemetry);
        speedMultiplier = oldMultiplier;
    }

    /**
     * Drive with robot-centric inputs.
     *
     * @param forward  Forward power [-1, 1], positive = forward
     * @param strafe   Strafe power [-1, 1], positive = left
     * @param rotate   Rotation power [-1, 1], positive = CCW
     */
    public void driveRobotCentric(double forward, double strafe, double rotate) {
        // Mecanum drive equations
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        // Normalize if any power exceeds 1.0
        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                Math.max(Math.abs(lb), Math.abs(rb))));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        setPowers(lf, rf, lb, rb);
    }

    /**
     * Drive with field-centric inputs.
     */
    public void driveFieldCentric(double forward, double strafe, double rotate, double heading) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotatedForward = forward * cos - strafe * sin;
        double rotatedStrafe = forward * sin + strafe * cos;

        driveRobotCentric(rotatedForward, rotatedStrafe, rotate);
    }

    // ==================== AUTONOMOUS POWER OUTPUT ====================

    /**
     * Set motor powers directly.
     * Order: [LF, RF, LB, RB]
     */
    public void setPowers(double lf, double rf, double lb, double rb) {
        // Clamp powers
        lf = clamp(lf, -1.0, 1.0);
        rf = clamp(rf, -1.0, 1.0);
        lb = clamp(lb, -1.0, 1.0);
        rb = clamp(rb, -1.0, 1.0);

        // Set motor powers
        LeftFront.setPower(lf);
        RightFront.setPower(rf);
        LeftRear.setPower(lb);
        RightRear.setPower(rb);
    }

    /**
     * Set motor powers from array.
     * Order: [LF, RF, LB, RB]
     */
    public void setPowers(double[] powers) {
        if (powers == null || powers.length != 4) {
            stop();
            return;
        }
        setPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    // ==================== CONFIGURATION ====================

    /**
     * Set speed multiplier for TeleOp.
     */
    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = Math.max(0.1, Math.min(1.0, multiplier));
    }

    /**
     * Set turn multiplier (rotation sensitivity relative to translation).
     */
    public void setTurnMultiplier(double multiplier) {
        this.turnMultiplier = Math.max(0.1, Math.min(1.0, multiplier));
    }

    /**
     * Set joystick deadzone.
     */
    public void setDeadzone(double dz) {
        this.deadzone = Math.max(0, Math.min(0.3, dz));
    }

    /**
     * Enable/disable field-centric driving.
     */
    public void setFieldCentric(boolean enabled) {
        this.fieldCentric = enabled;
    }

    /**
     * Set heading supplier for field-centric driving.
     */
    public void setHeadingSupplier(java.util.function.DoubleSupplier supplier) {
        this.headingSupplier = (supplier != null) ? supplier : () -> 0.0;
    }

    /**
     * Toggle driving direction (front/back swap).
     */
    public void toggleDirection() {
        directionMultiplier *= -1.0;
    }

    /**
     * Set driving direction multiplier.
     */
    public void setDirectionMultiplier(double mult) {
        this.directionMultiplier = Math.signum(mult);
        if (this.directionMultiplier == 0) this.directionMultiplier = 1.0;
    }

    // ==================== MOTOR DIRECTION CONFIGURATION ====================

    /**
     * Set individual motor direction.
     */
    public void setMotorDirection(int motorIndex, DcMotorSimple.Direction direction) {
        if (motorIndex >= 0 && motorIndex < 4) {
            motors.get(motorIndex).setDirection(direction);
        }
    }

    /**
     * Reverse a motor (toggle its direction).
     */
    public void reverseMotor(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < 4) {
            DcMotorEx motor = motors.get(motorIndex);
            if (motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }
    }

    // ==================== TELEMETRY ====================

    /**
     * Add drive telemetry.
     */
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Speed Mult", "%.2f", speedMultiplier);
        telemetry.addData("Direction", directionMultiplier > 0 ? "NORMAL" : "REVERSED");
        telemetry.addData("Field Centric", fieldCentric);

        telemetry.addLine("--- Motors ---");
        telemetry.addData("LF Power", "%.2f", LeftFront.getPower());
        telemetry.addData("RF Power", "%.2f", RightFront.getPower());
        telemetry.addData("LB Power", "%.2f", LeftRear.getPower());
        telemetry.addData("RB Power", "%.2f", RightRear.getPower());

        if (battery != null) {
            telemetry.addData("Battery", "%.2fV", battery.getVoltage());
        }
    }

    /**
     * Get current motor powers.
     */
    public double[] getPowers() {
        return new double[]{
                LeftFront.getPower(),
                RightFront.getPower(),
                LeftRear.getPower(),
                RightRear.getPower()
        };
    }

    /**
     * Get current motor positions (encoder ticks).
     */
    public int[] getPositions() {
        return new int[]{
                LeftFront.getCurrentPosition(),
                RightFront.getCurrentPosition(),
                LeftRear.getCurrentPosition(),
                RightRear.getCurrentPosition()
        };
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Apply deadzone to joystick input.
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        // Scale remaining range to [0, 1]
        double sign = Math.signum(value);
        double magnitude = (Math.abs(value) - deadzone) / (1.0 - deadzone);
        return sign * magnitude;
    }

    /**
     * Clamp value to range.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Get battery voltage (or default if unavailable).
     */
    public static double getBatteryVoltage() {
        if (battery != null) {
            double v = battery.getVoltage();
            if (Double.isFinite(v) && v > 0) {
                return v;
            }
        }
        return DriveConstants.NOMINAL_BATTERY_VOLT;
    }
}