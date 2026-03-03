package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
@TeleOp(name = "Color & Proximity Sensor Test", group = "OpModes")
public class RangefinderTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    FtcDashboard dashboard;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        dashboard = FtcDashboard.getInstance();

        robot.initializeHardware(hardwareMap);
        robot.initialize();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("Green?", robot.pin1.getState());
        telemetry.addData("Purple?", robot.pin0.getState());
        telemetry.addData("In front of?", robot.proximitySensor.getState());
        telemetry.update();
    }
}
