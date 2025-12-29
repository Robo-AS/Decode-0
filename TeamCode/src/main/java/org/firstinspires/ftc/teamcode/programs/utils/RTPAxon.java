package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees
    private double targetRotation;
    private double encoderZero = 0;
    private final double gearRatio = 1.5;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }

    // region constructors

    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }

    public void updatePIDCoeffs(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        resetPID();
    }

    // Initialization logic for servo and encoder
    public void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) { }

        // Try to get a valid starting position
        do {
            STARTPOS = getRawAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getRawAngle();
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ignored) { }
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        encoderZero = previousAngle;

        totalRotation = 0;
        homeAngle = 0;   // now home angle is always 0 due to encoderZero subtraction

        // Default PID coefficients
        kP = 0.015;
        kI = 0.0005;
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
        cliffs = 0;
    }
    // endregion

    // INTERNAL RAW ENCODER VALUE (never apply correction here)
    private double getRawAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) *
                (direction.equals(Direction.REVERSE) ? -360 : 360);
    }

    // Set servo direction
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    // Set power to servo, respecting direction and maxPower
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getPower() { return power; }

    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    public double getMaxPower() { return maxPower; }

    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) resetPID();
    }

    public boolean getRtp() { return rtp; }

    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; resetIntegral(); }
    public void setKD(double kD) { this.kD = kD; }

    public void setPidCoeffs(double kP, double kI, double kD) {
        setKP(kP); setKI(kI); setKD(kD);
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }

    public void setK(double k) { setKP(k); }
    public double getK() { return getKP(); }

    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }
    public double getMaxIntegralSum() { return maxIntegralSum; }

    public double getTotalRotation() { return totalRotation; }
    public double getTargetRotation() { return targetRotation; }

    public void changeTargetRotation(double change) { targetRotation += change; }

    public void setTargetRotation(double target) {
        targetRotation = target;
        resetPID();
    }

    public double getCurrentAngle() {
        double encoderAngle = getRawAngle() - encoderZero;
        return encoderAngle / gearRatio;
    }

    public boolean isAtTarget() { return isAtTarget(5); }
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    public void resetIntegral() { integralSum = 0; }

    // Main update loop
    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        // Handle wraparound at 0/360 degrees
        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        // Update total rotation
        totalRotation = (currentAngle) + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.001 || dt > 1.0) return;

        double error = targetRotation - totalRotation;

        // PID integral
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) integralSum *= 0.95;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        final double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double power = -Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(power);
        } else {
            setPower(0);
        }
    }

    // Inside RTPAxon.java
    public void setTargetAngleNearest(double targetAngle) {
        double current = totalRotation; // This is the cumulative angle
        // Calculate the shortest distance to the target angle
        double delta = ((targetAngle - current + 180) % 360 + 360) % 360 - 180;
        this.targetRotation = current + delta;
        // Do NOT call resetPID() here if you call this every loop,
        // it will wipe your integral sum and derivative.
    }
}
