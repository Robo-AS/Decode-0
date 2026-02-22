package org.firstinspires.ftc.teamcode.programs.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public Mecanum mecanum = null;
    public Limelight3A limelight = null;
    public TurretCR turret = null;
    public RTPAxon axon = null;
    public Flywheel flywheel = null;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, intakeFront, intakeBack;
    public DcMotorEx launcher1, launcher2;
    public Servo servoY, servoBarrier, servoIntake;
    public CRServo servoX;
    public List<DcMotorEx> motors;
    public static MultipleTelemetry telemetry;
    public GoBildaPinpointDriver pinpoint;

    public double x = 0, y = 0, heading = 0, lastTurretAngle = 0, driverOffset = 0;
    public boolean shootFar = false, limelightOnlyAim = false, wasUpperBlueAutoRan = false;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(131.16, -6.717, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        Robot.hardwareMap = hardwareMap;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        servoX = hardwareMap.get(CRServo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");

        servoX.setDirection(DcMotorSimple.Direction.REVERSE);

        axon = new RTPAxon(servoX, intakeBack);
        turret = new TurretCR();

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel = new Flywheel();
        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void initialize() {
        if (mecanum != null) mecanum.initialize();
        configurePinpoint();
        if (flywheel != null) flywheel.initialize();
        if (axon != null) axon.initialize(lastTurretAngle);
        if (turret != null) turret.initialize();
        if (servoY != null) servoY.setPosition(1);
        if (servoBarrier != null) servoBarrier.setPosition(0.35);
    }

    public void initializeHardwareAuto(HardwareMap hardwareMap) {
        Robot.hardwareMap = hardwareMap;
      //  limelight = hardwareMap.get(Limelight3A.class, "limelight");
      //  limelight.start();

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoY = hardwareMap.get(Servo.class, "servoY");
        servoX = hardwareMap.get(CRServo.class, "servoX");

        servoX.setDirection(DcMotorSimple.Direction.REVERSE);

        axon = new RTPAxon(servoX, intakeBack);

        turret = new TurretCR();
        flywheel = new Flywheel();

        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
    }

    public void initializeAuto() {
        if (flywheel != null) flywheel.initialize();
        if (turret != null) turret.initialize();
        if(axon != null) axon.initialize(0);
    }

    public void update() {
        if (pinpoint != null) pinpoint.update();
    }

    public Mecanum getInstanceMecanum(){
        return (mecanum == null) ? new Mecanum() : mecanum;
    }

    public TurretCR getInstanceTurret(){
        return (turret == null) ? new TurretCR() : turret;
    }

    public Flywheel getInstanceFlywheel(){
        if(flywheel == null) flywheel = new Flywheel();
        return flywheel;
    }

    public static HardwareMap getInstanceHardwareMap(){
        return hardwareMap;
    }

    public void initializePinpoint(HardwareMap hardwareMap){
        Robot.hardwareMap = hardwareMap;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
    }

    public static void clearInstance() {
        instance = null;
    }

}