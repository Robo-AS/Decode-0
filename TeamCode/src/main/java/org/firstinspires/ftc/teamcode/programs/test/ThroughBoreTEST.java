package org.firstinspires.ftc.teamcode.programs.test;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ThroughBoreTEST", group = "Tests")
public class ThroughBoreTEST extends CommandOpMode {

    public DcMotorEx intakeBack;
    public double position = 0;
    private final double TICKS_PER_REV = 8192.0;


    @Override
    public void initialize() {
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void run(){
        position = (intakeBack.getCurrentPosition() / TICKS_PER_REV) * 360.0;

        telemetry.addData("Position", position);
        telemetry.update();
    }

}
