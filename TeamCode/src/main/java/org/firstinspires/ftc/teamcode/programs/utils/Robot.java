package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public Mecanum mecanum = null;
    public Limelight limelight = null;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        mecanum = new Mecanum();
        limelight = new Limelight();
    }

    public void initialize()
    {
        limelight.initialize();
        limelight.initializeHardware(hardwareMap);
        mecanum.initialize();
    }

    public Mecanum getInstanceMecanum(){
        if(mecanum == null)
            return new Mecanum();
        return mecanum;
    }

    public Limelight getInstanceLimelight(){
        if(limelight == null)
            return new Limelight();
        return limelight;
    }

    public static HardwareMap getInstanceHardwareMap(){
        return hardwareMap;
    }
}