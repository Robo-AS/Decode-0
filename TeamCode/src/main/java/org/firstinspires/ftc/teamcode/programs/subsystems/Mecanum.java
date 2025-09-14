package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Mecanum  {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    public void initialize() {
        Robot robot = Robot.getInstance();

        this.leftFront = robot.leftFront;
        this.leftRear  = robot.leftRear;
        this.rightRear = robot.rightRear;
        this.rightFront = robot.rightFront;
    }

    public void drive(double strafe, double forward, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        leftFront.setPower(fl / max);
        rightFront.setPower(fr / max);
        leftRear.setPower(bl / max);
        rightRear.setPower(br / max);
    }
}

