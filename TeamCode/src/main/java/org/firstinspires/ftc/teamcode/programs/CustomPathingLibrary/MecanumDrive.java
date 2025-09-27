package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import static org.firstinspires.ftc.teamcode.programs.utils.TeleOpConstants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.programs.utils.TeleOpConstants.ROBOT_SPEED;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive {
    public DcMotorEx LeftFront, RightFront, LeftRear, RightRear;
    private List <DcMotorEx> motors;
    double reverse = 1.0;
    public static double powerReduction = 20;

    public MecanumDrive(HardwareMap hardwareMap) {
        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotorEx.class, "LeftRear");
        RightRear = hardwareMap.get(DcMotorEx.class, "RightRear");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        motors = Arrays.asList(LeftFront, LeftRear, RightRear, RightFront);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void teleop(GamepadEx gamepad, Telemetry telemetry) {
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double r = -gamepad.getRightX();

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double LeftFrontPower = (y + x + r);
        double RightFrontPower = (y - x - r);
        double LeftRearPower = (y - x + r);
        double RightRearPower = (y + x - r);

        LeftFront.setPower(clip(LeftFrontPower));
        RightFront.setPower(clip(RightFrontPower));
        LeftRear.setPower(clip(LeftRearPower));
        RightRear.setPower(clip(RightRearPower));
    }
    public void Slow_Motion(GamepadEx gamepad, Telemetry telemetry){
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double r = -gamepad.getRightX();

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double LeftFrontPower = (y + x + r);
        double RightFrontPower = (y - x - r);
        double LeftRearPower = (y - x + r);
        double RightRearPower = (y + x - r);

        LeftFront.setPower(clip(LeftFrontPower/powerReduction));
        RightFront.setPower(clip(RightFrontPower/powerReduction));
        LeftRear.setPower(clip(LeftRearPower/powerReduction));
        RightRear.setPower(clip(RightRearPower/powerReduction));
    }

    public void setPowers(double lf, double rf, double lb, double rb)
    {
        LeftFront.setPower(clip(lf/powerReduction));
        RightFront.setPower(clip(rf/powerReduction));
        LeftRear.setPower(clip(lb/powerReduction));
        RightRear.setPower(clip(rb/powerReduction));
    }
    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }
    double clip(double power) {
        return Math.max(-1.0, Math.min(power, 1.0));
    }
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", ROBOT_SPEED);

        telemetry.addData("LeftRear Position: ", LeftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", RightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", LeftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", RightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", LeftRear.getPower());
        telemetry.addData("RightRear Power: ", RightRear.getPower());
        telemetry.addData("LeftFront Power: ", LeftFront.getPower());
        telemetry.addData("RightFront Power: ", RightFront.getPower());
    }
}