package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import com.qualcomm.robotcore.hardware.CRServo;

import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

@Config
@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode {

    Robot robot = Robot.getInstance();
    CRServo servoX;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double targetAngle = 0.0;

    public static double GEAR_RATIO = 1.5;
    private double encoderZero = 0.0;

    PIDFController pid = new PIDFController(kP, kI, kD, 0);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV);

    FtcDashboard dashboard;

    @Override
    public void init() {
        robot.initializeHardware(hardwareMap);
        servoX = robot.servoX;
        dashboard = FtcDashboard.getInstance();
    }

    private double getRawAngle() {
        double pct = robot.axonEncoder.getVoltage() / robot.axonEncoder.getMaxVoltage();
        return pct * 360.0 / GEAR_RATIO;
    }

    private double getAngle() {
        double angle = getRawAngle() - encoderZero;

        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;

        return angle;
    }

    public void loop() {
        double currentAngle = getAngle();

        pid.setPIDF(kP, kI, kD, 0);
        ff = new SimpleMotorFeedforward(kS, kV);

        double pidOut = pid.calculate(currentAngle, targetAngle);
        double ffOut = ff.calculate(targetAngle - currentAngle);
        double power = pidOut + ffOut;

        power = Math.max(-1.0, Math.min(1.0, power));

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentAngle", currentAngle);
        packet.put("targetAngle", targetAngle);
        packet.put("power", power);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Current Angle", currentAngle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}
