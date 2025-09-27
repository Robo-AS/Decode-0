package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Dead-wheel (odometry) encoder helper. All distances are inches. */
public class Encoder {
    private final DcMotorEx motor;
    private Direction direction = Direction.FORWARD;

    // --- set these from robot spec / calibration ---
    public static double TICKS_PER_REV = 8192.0;  // e.g., REV Through-Bore
    public static double WHEEL_RADIUS_IN = 1.0;   // odometry wheel radius (in)
    public static double GEAR_RATIO = 1.0;        // 1 if directly coupled
    // ----------------------------------------------------

    private int lastTicks = 0;
    private boolean first = true;

    public enum Direction { FORWARD(1), REVERSE(-1); final int sign; Direction(int s){sign=s;} }

    public Encoder(DcMotorEx motor){
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDirection(Direction dir){ this.direction = dir; }

    public int rawTicks(){ return motor.getCurrentPosition(); }

    public double ticksToInches(int ticks){
        double revs = ticks / (TICKS_PER_REV * GEAR_RATIO);
        return revs * (2.0 * Math.PI * WHEEL_RADIUS_IN);
    }

    /** Absolute distance since last reset (inches, signed by direction). */
    public double getCurrentPositionIn(){
        return ticksToInches(rawTicks() * direction.sign);
    }

    /** Delta inches since last call; first call returns 0 and initializes. */
    public double deltaInches(){
        int t = rawTicks() * direction.sign;
        if (first){ first=false; lastTicks=t; return 0.0; }
        int dticks = t - lastTicks;
        lastTicks = t;
        return ticksToInches(dticks);
    }
}
