package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.solverslib.controller.PIDFController;
import com.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.solverslib.util.InterpLUT;

@Config
public class Flywheel extends OpMode {
    InterpLUT dist =new InterpLUT();
    InterpLUT vel = new InterpLUT();
    PIDFController Feedback;
    SimpleMotorFeedforward feedforward;
    static double kP = 0;
    static double kI =0;
    static double kD = 0;
    static double kS = 0;
    static double kV = 0;
    double length;// detected by camera
    double f = 0;// placholder cus in the loop it changes the f to the feedforward output
    DcMotorEx flyWheel;
    @Override
    public void init() {


        feedforward = new SimpleMotorFeedforward(kS,kV);
        Feedback = new PIDFController(kP,kI,kD,f);
        flyWheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        distLUTinit();
    }

    @Override
    public void loop() {
        Feedback.setPIDF(kP,kI,kD,f);
        double distance = dist.get(length);// gonna use camera for this so ill update this later
        double velocity = vel.get(distance);// gonna be based on distance or smt
        Feedback.setF(feedforward.calculate(flyWheel.getVelocity(),velocity));
        double power = Feedback.calculate(flyWheel.getVelocity(),velocity);



        flyWheel.setPower(power);

    }
    public void distLUTinit(){
        double pixelLength =0;//placeholders
        double distance = 0;
        dist.add(pixelLength,distance);// prob not gonna be a variable,
        // and also have mutliple add statements
        dist.createLUT();
    }
    public void velocityLUTinit(){
        double distance = 0;
        double velocity = 0;
        vel.add(distance,velocity);
        vel.createLUT();

    }

}