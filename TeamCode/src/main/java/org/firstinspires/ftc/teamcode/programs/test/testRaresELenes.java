package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.SetIntakeState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Se invart axoanele si ma suge Rares", group = "OpModes")
public class testRaresELenes extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public DcMotorEx intakeBack;
    public CRServo turretServo, turretServo2;

    public double position = 0;
    private final double TICKS_PER_REV = 8192.0;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServo = hardwareMap.get(CRServo.class, "servoX");
        turretServo2 = hardwareMap.get(CRServo.class, "servoTurret");

        turretServo2.setDirection(DcMotorSimple.Direction.FORWARD);
        turretServo.setDirection(DcMotorSimple.Direction.FORWARD);


        turretServo.setPower(0);
        turretServo2.setPower(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        turretServo.setPower(0.3);
        turretServo2.setPower(0.3);

        position = (intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;

        telemetry.addData("Position", position);
        telemetry.update();
    }
}