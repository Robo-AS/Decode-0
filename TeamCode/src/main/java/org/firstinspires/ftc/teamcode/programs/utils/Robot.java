package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hood;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public List<LynxModule> allHubs;
    public Mecanum mecanum = null;
    public Limelight3A limelight = null;
    public TurretCR turret = null;
    public Flywheel flywheel = null;
    public Hood hood;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, intakeFront, intakeBack;
    public DcMotorEx launcher1, launcher2;
    public Servo hoodServo, servoBarrier, servoIntake, servoSorter, lockMecanum_1, lockMecanum_2;
    public CRServo turretServo, sec_turretServo;
    public List<DcMotorEx> motors;
    public GoBildaPinpointDriver pinpoint;
    public TouchSensor backArtefacts, frontArtefacts, are3Artefacts_1, are3Artefacts_2;
    public DigitalChannel proximitySensor, pin0, pin1;
    public Servo led;
    public Intake intake;
    public double x = 0, y = 0, heading = 0, lastTurretAngle = 0, driverOffset = 0;
    public boolean shootFar = false, limelightOnlyAim = false, wasUpperBlueAutoRan = false;

    public static Robot getInstance() {
        if (instance == null) instance = new Robot();
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap) {
        Robot.hardwareMap = hardwareMap;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        turretServo = hardwareMap.get(CRServo.class, "servoX");
        sec_turretServo = hardwareMap.get(CRServo.class, "servoTurret");
        hoodServo = hardwareMap.get(Servo.class, "servoY");
        turretServo.setDirection(DcMotorSimple.Direction.FORWARD);
        sec_turretServo.setDirection(DcMotorSimple.Direction.FORWARD);


        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);


        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
        servoIntake.setDirection(Servo.Direction.REVERSE
        );

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        led = hardwareMap.get(Servo.class, "led");

        are3Artefacts_1 = hardwareMap.get(TouchSensor.class, "artefact1");
        are3Artefacts_2 = hardwareMap.get(TouchSensor.class, "artefact2");
        backArtefacts = hardwareMap.get(TouchSensor.class, "backArtefacts");
        frontArtefacts = hardwareMap.get(TouchSensor.class, "frontArtefacts");

        proximitySensor = hardwareMap.get(DigitalChannel.class, "proximitySensor");
        proximitySensor.setMode(DigitalChannel.Mode.INPUT);
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");

        servoSorter = hardwareMap.get(Servo.class, "servoSorter");

        lockMecanum_1 = hardwareMap.get(Servo.class, "lock1");
        lockMecanum_2 = hardwareMap.get(Servo.class, "lock2");

        intake = new Intake();
        mecanum = new Mecanum();
        turret = new TurretCR();
        flywheel = new Flywheel();
        hood = new Hood();
    }


    public void initialize() {
        if (mecanum != null) mecanum.initialize();
        configurePinpoint();
        if (flywheel != null) flywheel.initialize();
        if (turret != null) turret.initialize();
        if (servoBarrier != null) servoBarrier.setPosition(0.35);
        if (servoIntake != null) servoIntake.setPosition(0);
        if (led != null) led.setPosition(0.475);
        hood.initialize();
        servoSorter.setPosition( 0.61 );
    }

    public void initializeHardwareAuto(HardwareMap hardwareMap) {
        Robot.hardwareMap = hardwareMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        turretServo = hardwareMap.get(CRServo.class, "servoX");
        sec_turretServo = hardwareMap.get(CRServo.class, "servoTurret");
        hoodServo = hardwareMap.get(Servo.class, "servoY");
        turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
        sec_turretServo.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoBarrier = hardwareMap.get(Servo.class, "servoBarrier");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");

        led = hardwareMap.get(Servo.class, "led");

        are3Artefacts_1 = hardwareMap.get(TouchSensor.class, "artefact1");
        are3Artefacts_2 = hardwareMap.get(TouchSensor.class, "artefact2");
        backArtefacts = hardwareMap.get(TouchSensor.class, "backArtefacts");
        frontArtefacts = hardwareMap.get(TouchSensor.class, "frontArtefacts");

        proximitySensor = hardwareMap.get(DigitalChannel.class, "proximitySensor");
        proximitySensor.setMode(DigitalChannel.Mode.INPUT);
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");

        servoSorter = hardwareMap.get(Servo.class, "servoSorter");

        intake = new Intake();
        mecanum = new Mecanum();
        turret = new TurretCR();
        flywheel = new Flywheel();
        hood = new Hood();
    }

    public void initializeAuto() {
        if (flywheel != null) flywheel.initialize();
        if (turret != null) turret.initialize();
        servoSorter.setPosition( 0.59);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(186.7, -0.5393, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void loop() {
         pinpoint.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public static void clearInstance() {
        instance = null;
    }
}