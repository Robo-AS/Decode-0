package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode {

    private Robot robot;
    private TurretCR turret;
    private FtcDashboard dashboard;

    // Local dashboard controls
    public static double kP = 0.0125;
    public static double kI = 0.0;
    public static double kD = 0.0005;
    public static double kS = 0.075;
    public static double targetAngle = 0.0;

    @Override
    public void init() {
        robot = Robot.getInstance();
        robot.initializeHardware(hardwareMap);

        // Reference the subsystem already created in your Robot class
        this.turret = robot.turret;
        turret.initialize();

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TurretCR.kP = kP;
        TurretCR.kI = kI;
        TurretCR.kD = kD;
        TurretCR.kS = kS;
        TurretCR.targetTurretPosition = targetAngle;

        CommandScheduler.getInstance().run();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentAngle", turret.getCurrentAngle());
        packet.put("targetAngle", TurretCR.targetTurretPosition);
        packet.put("error", TurretCR.targetTurretPosition - turret.getCurrentAngle());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Current Angle", turret.getCurrentAngle());
        telemetry.addData("Target Angle", TurretCR.targetTurretPosition);
        telemetry.update();
    }

    @Override
    public void stop() {
        TurretCR.targetTurretPosition = turret.getCurrentAngle();
    }
}