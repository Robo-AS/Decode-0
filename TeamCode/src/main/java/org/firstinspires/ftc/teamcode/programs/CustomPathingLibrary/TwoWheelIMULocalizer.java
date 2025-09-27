package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;


/** Two dead wheels (parallel & perpendicular) + IMU heading. All inches/radians. */
public class TwoWheelIMULocalizer {

    // Tunables: odometry wheel frame offsets (inches) relative to robot center.
    // +x forward, +y left. If wheel lies exactly at centerlines, leave 0.
    public static double PERP_X_OFFSET_IN = 0.0;  // perp wheel x offset
    public static double PAR_Y_OFFSET_IN  = 0.0;  // parallel wheel y offset

    // Low-pass filter for velocities (0 = none, 1 = heavy smoothing)
    private final double velAlpha;

    private final Encoder parallel;     // forward distance sensor
    private final Encoder perpendicular;// left distance sensor
    private final IMUHelper imu;

    private Pose2d pose = new Pose2d(0,0,0);
    private double lastHeading = 0.0;
    private boolean first = true;

    // velocity estimates (field frame)
    private double vx = 0.0, vy = 0.0, omega = 0.0;

    public TwoWheelIMULocalizer(Encoder parallel, Encoder perpendicular, IMUHelper imu, double velFilterAlpha){
        this.parallel = parallel;
        this.perpendicular = perpendicular;
        this.imu = imu;
        this.velAlpha = Math.max(0.0, Math.min(1.0, velFilterAlpha));
        this.lastHeading = imu.headingRad();
    }

    public void setPose(Pose2d p){
        this.pose = p;
        this.lastHeading = imu.headingRad();
        this.vx=0; this.vy=0; this.omega=0;
        // also zero encoder deltas
        parallel.deltaInches();
        perpendicular.deltaInches();
        first = false;
    }

    public Pose2d getPose(){ return pose; }
    public double getVx(){ return vx; }
    public double getVy(){ return vy; }
    public double getOmega(){ return omega; }

    /** Call every loop with dt in seconds. */
    public void update(double dt){
        if (dt <= 0) dt = 1e-3;

        // IMU heading
        double heading = imu.headingRad();
        double dHeading = heading - lastHeading;
        // normalize to shortest
        while (dHeading <= -Math.PI) dHeading += 2*Math.PI;
        while (dHeading >   Math.PI) dHeading -= 2*Math.PI;

        // Encoder deltas (robot frame displacements)
        double dPar  = parallel.deltaInches();
        double dPerp = perpendicular.deltaInches();

        // Correct for rotation if wheels are offset from center (first-order)
        // If heading increases CCW, a wheel at positive offset travels s = offset * dHeading
        double dParRot  = -PAR_Y_OFFSET_IN  * dHeading; // parallel wheel sees motion from rotation about y-offset
        double dPerpRot =  PERP_X_OFFSET_IN * dHeading; // perpendicular wheel sees motion from rotation about x-offset
        double dx_r = dPar  + dParRot;   // +x forward
        double dy_r = dPerp + dPerpRot;  // +y left

        // Use mid-heading during interval
        double hMid = lastHeading + 0.5 * dHeading;
        double sh = Math.sin(hMid), ch = Math.cos(hMid);

        // Rotate robot-frame delta to field frame
        double dx_f =  ch*dx_r - sh*dy_r;
        double dy_f =  sh*dx_r + ch*dy_r;

        // Integrate
        pose.x += dx_f;
        pose.y += dy_f;
        pose.heading = wrap(heading);

        // Velocities (low-pass filtered)
        double vxNew = dx_f / dt;
        double vyNew = dy_f / dt;
        double wNew  = dHeading / dt;

        vx = (1.0 - velAlpha)*vxNew + velAlpha*vx;
        vy = (1.0 - velAlpha)*vyNew + velAlpha*vy;
        omega = (1.0 - velAlpha)*wNew + velAlpha*omega;

        lastHeading = heading;
        first = false;
    }

    private static double wrap(double a){
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }
}
