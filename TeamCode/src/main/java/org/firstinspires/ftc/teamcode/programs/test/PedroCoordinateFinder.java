package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.SetIntakeState;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.SetServoIntakeState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Pedro Pathing Coordinate Finder", group = "Test")
public class PedroCoordinateFinder extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Follower follower;

    private final Pose startPose = new Pose(21.010, 124.010, Math.toRadians(144));

    public static double constantTerm = 0.6;
    public static double liniarCoefTerm = 0.7;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // We initialize non-drivetrain hardware (like slides/intake links if needed)
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        // Initialize follower and let it configure its own motor mappings
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Ensure the follower starts in TeleOp mode
        follower.startTeleopDrive();

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new DoesNothingCommand(),
                                        new ParallelCommandGroup(
                                                new SetServoIntakeState(Intake.ServoIntakeState.DOWN),
                                                new SetIntakeState(Intake.IntakeState.ON)
                                        ),
                                        () -> robot.intake.servoIntakeState == Intake.ServoIntakeState.DOWN && robot.intake.intakeState == Intake.IntakeState.ON
                                )
                        )
                );



        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(
                        () -> CommandScheduler.getInstance().schedule(
                                new ParallelCommandGroup(
                                        new SetServoIntakeState(Intake.ServoIntakeState.UP),
                                        new SetIntakeState(Intake.IntakeState.OFF)
                                )
                        )
                );
    }

    @Override
    public void run() {
        // 1. Inputs mapped precisely to your shape preferences
        double x_input = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
        double y_input = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;
        double turn_input = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;

        // 2. Hand drive vectors directly to Pedro Pathing. 
        // This calculates the motor powers safely without interfering with internal PID states.
        // Arguments: (Drive X [forward], Drive Y [strafe], Turn, Robot-Centric true/false)
        follower.setTeleOpDrive(-y_input, -x_input, -turn_input, true);

        // 3. This updates the Pinpoint localizer and applies the motor vectors simultaneously
        follower.update();

        // 4. Read coordinates safely
        Pose currentPose = follower.getPose();

        telemetry.addLine("--- DRIVE TO TARGET POSITION ---");
        telemetry.addData("Live X Coordinate", currentPose.getX());
        telemetry.addData("Live Y Coordinate", currentPose.getY());
        telemetry.addData("Live Heading (Deg)", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Live Heading (Rad)", currentPose.getHeading());
        telemetry.addLine();
        telemetry.addLine("Code Formatting Hint:");
        telemetry.addLine(String.format("new Pose(%.3f, %.3f, Math.toRadians(%.1f))", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
        telemetry.update();
    }
}
