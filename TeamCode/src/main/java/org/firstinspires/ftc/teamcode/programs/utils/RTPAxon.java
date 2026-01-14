package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    private final AnalogInput servoEncoder;
    private final CRServo servo;
    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private double encoderZero = 0;
    private final double gearRatio = 1.5;

    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public RTPAxon(CRServo servo, AnalogInput encoder) {
        this.rtp = true;
        this.servo = servo;
        this.servoEncoder = encoder;
        this.direction = Direction.FORWARD;
        initialize(0);
    }

    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this.rtp = true;
        this.servo = servo;
        this.servoEncoder = encoder;
        this.direction = direction;
        initialize(0);
    }

    public void updatePIDCoeffs(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        resetPID();
    }

    public void initialize() {
        initialize(0);
    }

    public void initialize(double startingAngle) {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) { }

        ntry = 0;
        double raw;
        do {
            raw = getRawAngle();
            ntry++;
            if (ntry > 50) break;
        } while (Math.abs(raw) < 0.01);

        this.encoderZero = raw - (startingAngle * gearRatio);

        this.previousAngle = startingAngle;
        this.totalRotation = startingAngle;
        this.targetRotation = startingAngle;
        this.cliffs = 0;

        kP = 0.015;
        kI = 0.0005;
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
    }

    private double getRawAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) *
                (direction.equals(Direction.REVERSE) ? -360 : 360);
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getPower() { return power; }

    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }

    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) resetPID();
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
        lastError = 0;
        pidTimer.reset();
    }

    public double getCurrentAngle() {
        double encoderAngle = getRawAngle() - encoderZero;
        return encoderAngle / gearRatio;
    }

    public double getTotalRotation() { return totalRotation; }

    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        totalRotation = (currentAngle) + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.0001 || dt > 0.5) return;

        double error = targetRotation - totalRotation;
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        if (Math.abs(error) < 2.0) integralSum *= 0.95;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        if (Math.abs(error) > 0.5) {
            setPower(-output);
        } else {
            setPower(0);
        }
    }
}