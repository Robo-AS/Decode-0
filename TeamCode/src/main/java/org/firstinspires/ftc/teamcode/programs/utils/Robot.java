package org.firstinspires.ftc.teamcode.programs.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.LimelightWrapper;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public Mecanum mecanum = null;
    public Limelight3A limelight = null;
    public LimelightWrapper llwrapped = null;
    public TurretCR turret = null;
    public RTPAxon axon = null;
    public Flywheel flywheel = null;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, intake;
    public DcMotorEx launcher1, launcher2;
    public Servo servoY, servoLauncher;
    public CRServo servoX;
    public AnalogInput axonEncoder;
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

    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        llwrapped = new LimelightWrapper();
        llwrapped.initializeHardware(hardwareMap);
        llwrapped.initialize();

        // turret

        servoX = hardwareMap.get(CRServo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");

        axonEncoder = hardwareMap.get(AnalogInput.class, "encoder");

        axon = new RTPAxon(servoX, axonEncoder);
        axon.setDirection(RTPAxon.Direction.REVERSE);
        turret = new TurretCR();

        //launcher
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoLauncher = hardwareMap.get(Servo.class, "servoLauncher");

        flywheel = new Flywheel();

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        //pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initializeHardwareAuto(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoLauncher = hardwareMap.get(Servo.class, "servoLauncher");

        servoY = hardwareMap.get(Servo.class, "servoY");

        flywheel = new Flywheel();
    }


    public void initialize() {
        mecanum.initialize();
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        pinpoint.resetPosAndIMU();
        flywheel.initialize();
        servoLauncher.setPosition(0);
        turret.initialize();
    }

    public void initializeAuto() {
        flywheel.initialize();
        servoLauncher.setPosition(0);
    }

    public void update() {
        pinpoint.update();
    }
    public Mecanum getInstanceMecanum(){
        if(mecanum == null)
            return new Mecanum();
        return mecanum;
    }

    public TurretCR getInstanceTurret(){
        if(turret == null)
            return new TurretCR();
        return turret;
    }

    public LimelightWrapper getInstanceLimelight(){
        if(limelight == null) {
            llwrapped = new LimelightWrapper();
            llwrapped.initializeHardware(hardwareMap);
        }
        return llwrapped;
    }

    public Flywheel getInstanceFlywheel(){
        if(flywheel == null){
            flywheel = new Flywheel();
        }

        return flywheel;
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