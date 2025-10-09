package org.firstinspires.ftc.teamcode.programs.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.programs.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.Turret;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public Mecanum mecanum = null;
    public Limelight limelight = null;
    public Turret turret = null;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, intake;
    public DcMotorEx leftLauncher, rightLauncher;
    public Servo servoX, servoY;
    public IMU imu;
    public List<DcMotorEx> motors;
    public static MultipleTelemetry telemetry;
    public RevHubOrientationOnRobot revHubOrientationOnRobot = null;
    public static GoBildaPinpointDriver pinpoint = null;


    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap, MultipleTelemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //mecanum

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

        //limelight

        limelight = new Limelight();

        limelight.initializeHardware(hardwareMap);

        // turret

        servoX = hardwareMap.get(Servo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");

        servoX.setDirection(Servo.Direction.REVERSE);
        servoY.setDirection(Servo.Direction.REVERSE);

        turret = new Turret();

        //launcher
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");

     //   leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
     //   rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        //pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initialize() {
        limelight.initialize();
        mecanum.initialize();
        turret.initialize();
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }
    public Mecanum getInstanceMecanum(){
        if(mecanum == null)
            return new Mecanum();
        return mecanum;
    }

    public Turret getInstanceTurret(){
        if(turret == null)
            return new Turret();
        return turret;
    }

    public Limelight getInstanceLimelight(){
        if(limelight == null) {
            limelight = new Limelight();
            limelight.initializeHardware(hardwareMap);
        }
        return limelight;
    }

    public static HardwareMap getInstanceHardwareMap(){
        return hardwareMap;
    }
    public static MultipleTelemetry getInstanceTelemetry(){
        return telemetry;
    }

    public static GoBildaPinpointDriver getInstancePinpoint() {
        return pinpoint;
    }
}
