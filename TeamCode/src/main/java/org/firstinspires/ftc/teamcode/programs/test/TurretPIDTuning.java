package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;

@Config
@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode {

    Robot robot = Robot.getInstance();
    RTPAxon turret;
    FtcDashboard dashboard;

    public static double kP = 0.0125;
    public static double kI = 0.0;
    public static double kD = 0.0005;
    public static double kS = 0.075;
    public static double targetAngle = 0.0;

    @Override
    public void init() {
        robot.initializeHardware(hardwareMap);
        turret.initialize(0);

        CommandScheduler.getInstance().unregisterSubsystem(robot.turret);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        turret.updatePIDCoeffs(kP, kI, kD, kS);

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

    @Override
    public void stop() {
        turret.setTargetRotation(turret.getCurrentAngle());
        turret.update();
    }
}