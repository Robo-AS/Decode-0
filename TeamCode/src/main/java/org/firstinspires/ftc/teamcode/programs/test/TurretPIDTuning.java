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
    public static double targetAngle = 0.0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        robot.initializeHardware(hardwareMap);
        turret = robot.axon;
        turret.initialize();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        turret.updatePIDCoeffs(kP, kI, kD);

        turret.setTargetRotation(targetAngle);

        turret.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentAngle", turret.getTotalRotation());
        packet.put("targetAngle", targetAngle);
        packet.put("power", turret.getPower());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Current Angle", turret.getTotalRotation());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Power", turret.getPower());
        telemetry.update();
    }
}
