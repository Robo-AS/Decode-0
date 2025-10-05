package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
import org.firstinspires.ftc.teamcode.programs.utils.geometry.Vector2D;

public class Mecanum  {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    double[] ws = new double[4];
    private final int frontLeft = 3, frontRight = 1, backLeft = 2, backRight = 0, ks = 0;

    public void initialize() {
        Robot robot = Robot.getInstance();

        this.leftFront = robot.leftFront;
        this.leftRear  = robot.leftRear;
        this.rightRear = robot.rightRear;
        this.rightFront = robot.rightFront;
    }

    public void set(PoseRR pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }

    public void set(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {

        Vector2D input = new Vector2D(strafeSpeed, forwardSpeed).rotate(-gyroAngle);
        double actualks = ks; // *12/getVoltage();


        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[frontLeft] = (forwardSpeed + strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed + strafeSpeed + turnSpeed));
        wheelSpeeds[frontRight] = (forwardSpeed - strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed - strafeSpeed - turnSpeed);
        wheelSpeeds[backLeft] = (forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed - strafeSpeed + turnSpeed);
        wheelSpeeds[backRight] = (forwardSpeed + strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed - turnSpeed);

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheelSpeeds[frontLeft] /= max;
            wheelSpeeds[frontRight] /= max;
            wheelSpeeds[backLeft] /= max;
            wheelSpeeds[backRight] /= max;
        }

        ws[frontLeft] = wheelSpeeds[frontLeft];
        ws[frontRight] = wheelSpeeds[frontRight];
        ws[backLeft] = wheelSpeeds[backLeft];
        ws[backRight] = wheelSpeeds[backRight];

        leftFront.setPower(ws[frontLeft]);
        rightFront.setPower(ws[frontRight]);
        leftRear.setPower(ws[backLeft]);
        rightRear.setPower(ws[backRight]);
    }
}

