package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

public final class DriveConstants {
    // Geometry (in)
    public static double TRACKWIDTH_IN = 14.0;
    public static double WHEELBASE_IN  = 14.0;

    // Limits
    public static double MAX_VEL_IN_S     = 40.0;
    public static double MAX_ACCEL_IN_S2  = 80.0;
    public static double MAX_DECEL_IN_S2  = 80.0;
    public static double MAX_JERK_IN_S3   = 600.0;
    public static double MAX_CENTRIPETAL  = 80.0; // a_lat limit (in/s^2)

    // Maximum angular velocity (rad/s)
// Estimate: max wheel linear speed (in/s) divided by half of trackwidth (in)
    public static final double MAX_ANG_VEL_RAD_S = (MAX_VEL_IN_S / (TRACKWIDTH_IN / 2.0));

    // Maximum angular acceleration (rad/s^2)
// You can tune this experimentally later
    public static final double MAX_ANG_ACCEL_RAD_S2 = 2.0 * Math.PI;  // ~1 rotation per second^2

    // Maximum angular jerk (rad/s^3) if you want jerk-limited turn profiles
    public static final double MAX_ANG_JERK_RAD_S3 = 10.0;  // tune later
    public static final double MAX_ANG_DECCEL_RAD_S2 = 2.0;


    // Wheel
    public static double MAX_WHEEL_SPEED_IN_S = 90.0; // = MAX_RPM / 60 * Wheel_Circumference

    // PIDF gains
    public static double KP_X=0.08, KI_X=0.0, KD_X=0.003, KF_X=0.0;
    public static double KP_Y=0.08, KI_Y=0.0, KD_Y=0.003, KF_Y=0.0;
    public static double KP_H=2.8,  KI_H=0.0, KD_H=0.02,  KF_H=0.0;

    // Derivative filter
    public static double LOOP_HZ    = 50.0;
    public static double D_CUTOFF_HZ= 20.0;

    // Slew limits on commands
    public static double MAX_DVX_IN_S2 = 150.0;
    public static double MAX_DVY_IN_S2 = 150.0;
    public static double MAX_DW_RAD_S2 = Math.toRadians(1200.0);

    private DriveConstants(){}
}
