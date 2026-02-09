package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    private final DcMotorEx encoderMotor;
    private final CRServo servo;
    private boolean rtp = true;
    private double power;
    private double maxPower = 0.4;
    private Direction direction = Direction.FORWARD;
    private double totalRotation;
    private double targetRotation;
    private final double TICKS_PER_REV = 8192.0;
    private final double gearRatio = 1.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kS = 0.0;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double maxIntegralSum = 1.0;
    private ElapsedTime pidTimer;

    private final double POSITION_TOLERANCE = 0.5;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public RTPAxon(CRServo servo, DcMotorEx encoderMotor) {
        this.servo = servo;
        this.encoderMotor = encoderMotor;
        initialize(0);
    }

    public void updatePIDCoeffs(double kP, double kI, double kD, double kS) {
        if (this.kP != kP || this.kI != kI || this.kD != kD || this.kS != kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            resetPID();
        }
    }

    public void initialize(double startingAngle) {
        servo.setPower(0);
        this.totalRotation = startingAngle;
        this.targetRotation = startingAngle;
        pidTimer = new ElapsedTime();
        pidTimer.reset();
        resetPID();
    }

    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getCurrentAngle() {
        double encoderDegrees = (encoderMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        if (direction == Direction.REVERSE) encoderDegrees *= -1;
        return encoderDegrees / gearRatio;
    }

    public void resetPID() {
        integralSum = 0;
        lastError = 0;
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    public double getPower() { return power; }

    public synchronized void update() {
        totalRotation = getCurrentAngle();

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.0001 || dt > 0.2) return;

        double error = targetRotation - totalRotation;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            integralSum += error * dt;
            integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

            double derivative = (error - lastError) / dt;
            lastError = error;

            double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
            double ffOutput = Math.signum(error) * kS;

            setPower(pidOutput + ffOutput);
        } else {
            setPower(0);
            resetPID();
        }
    }
}