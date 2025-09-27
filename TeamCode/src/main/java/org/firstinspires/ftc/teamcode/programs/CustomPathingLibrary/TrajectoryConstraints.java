package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public class TrajectoryConstraints {
    public double maxVel;          // in/s
    public double maxAccel;        // in/s^2
    public double maxDecel;        // in/s^2 (positive, applied as -maxDecel)
    public double maxJerk;         // in/s^3
    public double maxCentripetal;  // in/s^2 (a_lat)
    public double maxAngVel;
    public double maxAngAccel;
    public double maxAngDecel;
    public double maxAngJerk;

    public TrajectoryConstraints(double maxVel,double maxAccel,double maxDecel,double maxJerk,double maxCentripetal){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxJerk = maxJerk;
        this.maxCentripetal = maxCentripetal;
        this.maxAngVel = DriveConstants.MAX_ANG_VEL_RAD_S;
        this.maxAngAccel = DriveConstants.MAX_ANG_ACCEL_RAD_S2;
        this.maxAngJerk = DriveConstants.MAX_ANG_JERK_RAD_S3;
        this.maxAngDecel = DriveConstants.MAX_ANG_DECCEL_RAD_S2;
    }
}
