package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;


@Config
@TeleOp(name = "Servo Position Setter", group = "Tuning")
public class ServoPositionSetter extends OpMode {

    Robot robot = Robot.getInstance();

    public static double LIFT = 0.0;
    public static double HOOD = 0.0;
    FtcDashboard dashboard;

    @Override
    public void init() {
        robot.initializeHardware(hardwareMap);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        robot.servoLauncher.setPosition(LIFT);
        robot.servoY.setPosition(HOOD);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Lift Position", LIFT);
        packet.put("Hood Position", HOOD);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Lift Position", LIFT);
        telemetry.addData("Hood Position", HOOD);
        telemetry.update();
    }
}
