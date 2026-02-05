package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
import org.firstinspires.ftc.teamcode.programs.commandbase.limelight.setServoYPosition;

@Config
@TeleOp(name = "Servo Position Setter", group = "OpModes")
public class ServoPositionSetter extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public static double servoBarrierPos = 0.5, servoYPos = 0.5;

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

        robot.servoY.setPosition(servoYPos);
        robot.servoBarrier.setPosition(servoBarrierPos);

        telemetry.addData("Servo Y", servoYPos);
        telemetry.addData("Servo Launcher", servoBarrierPos);
        telemetry.update();
    }
}
