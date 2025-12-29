package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@TeleOp(name = "Flywheel PID Tuning", group = "OpModes")
public class FlywheelPIDTuning extends OpMode {
    InterpLUT vel = new InterpLUT();
    PIDFController pid_Flywheel;
    SimpleMotorFeedforward feedforward;
    private Limelight3A limelight;
    public double ty, y_distance;
    public static double kP = 0;
    public static double kI =0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kV = 0;

    public static double targetVelocity = 0, currentVelocity = 0;
    public double downY = 0.2, upY = 0.6, maxDistance = 0.004, minDistance = 0.2704;
    double f = 0;
    private DcMotorEx flyWheel1, flyWheel2;
    FtcDashboard dashboard;

    public double CAMERA_ANGLE = 18;
    public double CAMERA_HEIGHT = 0.4;

    @Override
    public void init()
    {
        Robot.getInstance().initializeHardware(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        feedforward = new SimpleMotorFeedforward(kS, kV);
        pid_Flywheel = new PIDFController(kP, kI, kD, f);

        flyWheel1 = Robot.getInstance().launcher1;
        flyWheel2 = Robot.getInstance().launcher2;
        limelight = Robot.getInstance().limelight;
    }

    @Override
    public void loop() {

        currentVelocity = flyWheel2.getVelocity();

        pid_Flywheel.setPIDF(kP, kI, kD, f);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        LLResult result = limelight.getLatestResult();

        if(result.isValid() && result != null) {
            ty = result.getTy();
            y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
            Robot.getInstance().servoY.setPosition(getServoYPositionFromDistance(y_distance));
        }

        double ff = feedforward.calculate(targetVelocity);
        double pid = pid_Flywheel.calculate(currentVelocity, targetVelocity);
        double power = pid + ff;

        flyWheel1.setPower(power);
        flyWheel2.setPower(power);

        // Send to Dashboard Graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentVelocity", currentVelocity);
        packet.put("targetVelocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet);
    }


    public double getServoYPositionFromDistance(double distance)
    {
        if(distance < maxDistance) return 0.6;
        if(distance > minDistance) return 0.2;

        double ratio = (minDistance - distance) / (minDistance - maxDistance);
        return downY + ratio * (upY - downY);
    }
}