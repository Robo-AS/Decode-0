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
    public static GoBildaPinpointDriver pinpoint = null;

    public double x = 0, y = 0, heading = 0, lastTurretAngle = 0;

    public boolean limelightAimOnly = false;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
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

        axon = new RTPAxon(servoX, intakeFront);
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
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

    }

    public void initializeHardwareAuto(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoY = hardwareMap.get(Servo.class, "servoY");
        servoX = hardwareMap.get(CRServo.class, "servoX");
        axon = new RTPAxon(servoX, intakeFront);

        turret = new TurretCR();
        flywheel = new Flywheel();

        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
    }

    public void initializeTurretHardware(HardwareMap hardwareMap){
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        servoX = hardwareMap.get(CRServo.class, "servoX");
        axon = new RTPAxon(servoX, intakeFront);
        turret = new TurretCR();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    public void initializeControlHub(HardwareMap hardwareMap){
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        servoY = hardwareMap.get(Servo.class, "servoY");
        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
    }

    public void initializeTest() {

        mecanum.initialize();

        pinpoint.resetPosAndIMU();

        // Encoder resolution (MUST be set every init)
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Pod offsets (measured from robot center)
        pinpoint.setOffsets(
                7.12,   // X pod is RIGHT of center → negative
                -130.6,    // Y pod is BEHIND center → negative
                DistanceUnit.MM
        );

        // Encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // ZERO pose for testing
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM,
                0,
                0,
                AngleUnit.RADIANS,
                0
        ));

        pinpoint.update();
    }

    public void initialize() {
        mecanum.initialize();

        mecanum.initialize();

        pinpoint.resetPosAndIMU();

        // Encoder resolution (MUST be set every init)
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Pod offsets (measured from robot center)
        pinpoint.setOffsets(
                7.12,   // X pod is RIGHT of center → negative
                -130.6,    // Y pod is BEHIND center → negative
                DistanceUnit.MM
        );

        // Encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // ZERO pose for testing
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM,
                0,
                0,
                AngleUnit.RADIANS,
                0
        ));

        pinpoint.update();

        flywheel.initialize();
        axon.initialize(lastTurretAngle);
        turret.initialize();
        servoY.setPosition(1);
        servoBarrier.setPosition(0.35);
    }

    public void initializeTurret(){
        mecanum.initialize();

        pinpoint.resetPosAndIMU();

        // Encoder resolution (MUST be set every init)
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Pod offsets (measured from robot center)
        pinpoint.setOffsets(
                7.12,   // X pod is RIGHT of center → negative
                -130.6,    // Y pod is BEHIND center → negative
                DistanceUnit.MM
        );

        // Encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // ZERO pose for testing
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM,
                0,
                0,
                AngleUnit.RADIANS,
                0
        ));

        pinpoint.initialize ();

        axon.initialize(lastTurretAngle);
        turret.initialize();
    }


    public void initializeAuto() {
        flywheel.initialize();
        turret.initialize();
    }

    public void update() {
        pinpoint.update();
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
    public static GoBildaPinpointDriver getInstancePinpoint() {
        return pinpoint;
    }

}