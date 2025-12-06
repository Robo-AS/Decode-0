package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;

@TeleOp(name = "Pinpoint Test", group = "OpModes")
public class PinpointTargetTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeHardware(hardwareMap);
        robot.initialize();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double turnInput = -((Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm);
        double forwardInput = -(Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        double strafeInput = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;
        PoseRR drive = new PoseRR(forwardInput, strafeInput, turnInput);
        robot.mecanum.set(drive, 0);

        robot.pinpoint.update();

        telemetry.addData("X", robot.pinpoint.getPosX(DistanceUnit.METER));
        telemetry.addData("Y", robot.pinpoint.getPosY(DistanceUnit.METER));
        telemetry.addData("Heading:", Math.toDegrees(robot.pinpoint.getHeading(AngleUnit.RADIANS)));
        telemetry.update();
    }
}