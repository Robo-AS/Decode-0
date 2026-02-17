package org.firstinspires.ftc.teamcode.programs.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RTPAxon {
    private final DcMotorEx encoderMotor;
    private final CRServo servo;
    private PIDController pid_axon;

    private boolean rtp = true;
    private double power;
    private double maxPower = 1;

    private Direction direction = Direction.FORWARD;

    private double targetRotation;
    private double kS = 0.0;
    private final double TICKS_PER_REV = 8192.0;
    private final double gearRatio = 1.0;
    private final double POSITION_TOLERANCE = 0.5;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public RTPAxon(CRServo servo, DcMotorEx encoderMotor) {
        this.servo = servo;
        this.encoderMotor = encoderMotor;
        this.pid_axon = new PIDController(0, 0, 0);
        initialize(0);
    }

    public void updatePIDCoeffs(double kP, double kI, double kD, double kS) {
        this.kS = kS;
        pid_axon.setPID(kP, kI, kD);
    }

    public void initialize(double startingAngle) {
        servo.setPower(0);
        this.targetRotation = startingAngle;
        pid_axon.reset();
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

    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    public void resetPID() {
        pid_axon.reset();
    }

    public synchronized void update() {
        if (!rtp) {
            servo.setPower(0);
            return;
        }

        double currentAngle = getCurrentAngle();
        double error = targetRotation - currentAngle;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            double pidOutput = pid_axon.calculate(currentAngle, targetRotation);
            double ffOutput = Math.signum(error) * kS;
            setPower(pidOutput + ffOutput);
        } else {
            setPower(0);
        }
    }
}