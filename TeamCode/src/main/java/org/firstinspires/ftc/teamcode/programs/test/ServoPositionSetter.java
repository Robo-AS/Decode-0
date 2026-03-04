package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
@TeleOp(name = "Servo Position Setter", group = "OpModes")
public class ServoPositionSetter extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public static double servoBarrierPos = 0.5, servoYPos = 0.5, servoIntakePos = 0.5, servoSorterPos = 0.5;
    //0.5 0.875 0.105
    FtcDashboard dashboard;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        dashboard = FtcDashboard.getInstance();

        robot.initializeHardware(hardwareMap);
        robot.initialize();
        robot.limelight.start();
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.hoodServo.setPosition(servoYPos);
        robot.servoBarrier.setPosition(servoBarrierPos);
        robot.servoIntake.setPosition(servoIntakePos);
        robot.servoSorter.setPosition(servoSorterPos);

        telemetry.addData("Servo Y", servoYPos);
        telemetry.addData("Servo Barrier", servoBarrierPos);
        telemetry.addData("Servo Intake", servoIntakePos);
        telemetry.addData("Servo Sorter", servoSorterPos);
        telemetry.update();
    }
}
