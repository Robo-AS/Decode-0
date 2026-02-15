package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode {

    Robot robot = Robot.getInstance();
    RTPAxon turret;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double targetAngle = 0.0;

    FtcDashboard dashboard;
    private double lastP = 0.0, lastI = 0.0, lastD = 0.0;

    @Override
    public void init() {
        robot.initializeHardware(hardwareMap);
        turret = robot.axon;
        turret.initialize(0);

        turret.updatePIDCoeffs(kP, kI, kD, kS);
        lastP = kP;
        lastI = kI;
        lastD = kD;

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        if (kP != lastP || kI != lastI || kD != lastD) {
            turret.updatePIDCoeffs(kP, kI, kD, kS);
            lastP = kP;
            lastI = kI;
            lastD = kD;
        }

        turret.setTargetRotation(targetAngle);
        turret.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentAngle", turret.getCurrentAngle());
        packet.put("targetAngle", targetAngle);
        packet.put("error", targetAngle - turret.getCurrentAngle());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Current Angle", turret.getCurrentAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.update();
    }
}