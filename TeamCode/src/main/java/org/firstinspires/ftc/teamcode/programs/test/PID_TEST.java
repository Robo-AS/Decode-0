package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;


@Config
@TeleOp(name = "PID_TEST", group = "Tests")
public class PID_TEST extends CommandOpMode {
//    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public DcMotorEx intakeBack;
    public CRServo turretServo, turretServo2;
    private final PIDController turretPID_RIGHT = new PIDController(kP_RIGHT, kI_RIGHT, kD_RIGHT);
    public static double kP_RIGHT = 0.006, kI_RIGHT = 0, kD_RIGHT = 0.0006, ks_RIGHT = 0.075;

    public static double targetTurretPosition = 0;
    public static double currentTurretPosition = 0;
    private final double TICKS_PER_REV = 8192.0;


    @Override
    public void initialize() {

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
        turretPID_RIGHT.reset();
        targetTurretPosition = 0;
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        targetTurretPosition = Range.clip(targetTurretPosition, -90, 360);

        currentTurretPosition = (intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        turretPID_RIGHT.setPID(kP_RIGHT, kI_RIGHT, kD_RIGHT);
        double error = targetTurretPosition - currentTurretPosition;
        double power = turretPID_RIGHT.calculate(currentTurretPosition, targetTurretPosition) + Math.signum(error) * ks_RIGHT;
        turretServo.setPower(power);
        turretServo2.setPower(power);


        telemetry.addData("Current Angle", currentTurretPosition);
        telemetry.addData("Target Angle", targetTurretPosition);
        telemetry.update();
    }
}
