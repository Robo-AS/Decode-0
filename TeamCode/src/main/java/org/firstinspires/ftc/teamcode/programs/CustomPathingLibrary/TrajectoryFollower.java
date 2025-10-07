package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import static java.lang.Math.*;

/** Time-parameterized trajectory follower with Frenet (tangent/normal) regulation + ω/α feedforward. */
public class TrajectoryFollower {

    public interface PoseSupplier { Pose2d getPose(); }

    private final PoseSupplier poseSupplier;
    private final MecanumDrive drive;

    // Tangent/Normal PIDF (instead of robot X/Y)
    private final AdvancedPIDF pidLong = new AdvancedPIDF(DriveConstants.KP_X, DriveConstants.KI_X, DriveConstants.KD_X, DriveConstants.KF_X);
    private final AdvancedPIDF pidLat  = new AdvancedPIDF(DriveConstants.KP_Y, DriveConstants.KI_Y, DriveConstants.KD_Y, DriveConstants.KF_Y);
    private final AdvancedPIDF pidH    = new AdvancedPIDF(DriveConstants.KP_H, DriveConstants.KI_H, DriveConstants.KD_H, DriveConstants.KF_H);

    // Slew limits on body-frame commands
    private final SlewRateLimiter sX = new SlewRateLimiter(DriveConstants.MAX_DVX_IN_S2);
    private final SlewRateLimiter sY = new SlewRateLimiter(DriveConstants.MAX_DVY_IN_S2);
    private final SlewRateLimiter sW = new SlewRateLimiter(DriveConstants.MAX_DW_RAD_S2);

    // Optional feedforward on angular acceleration (units: seconds)
    private double kFFAlpha = 0.0;   // set nonzero if you want α feedforward → ω command (e.g., 0.05–0.15 s)

    // Optional cross-track → heading coupling (rad/s per inch), helps “point into” the path when off it
    private double kEyToW = 0.0;     // try small values like 0.1–0.3 rad/(s·in)

    public Trajectory traj = null;
    private double t0 = 0.0;
    private double lastT = 0.0;

    // Keep last Frenet errors for D term (if you want explicit D; PIDF already computes internally from meas)
    private double lastElong = 0.0, lastElat = 0.0, lastEh = 0.0;
    // extras at top of class
    private static final double END_S_EPS = 0.5;       // in
    private static final double END_POS_EPS = 0.75;    // in
    private static final double END_HEADING_EPS = Math.toRadians(3.0); // rad

    // Cache end state when we load a trajectory
    public Trajectory.State endState = null;


    public TrajectoryFollower(PoseSupplier supplier, MecanumDrive drive){
        this.poseSupplier = supplier;
        this.drive = drive;

        // filters/limits matching your constants
        pidLong.setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);
        pidLat .setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);
        pidH   .setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);

        pidLong.setIntegralLimit(2.0);
        pidLat .setIntegralLimit(2.0);
        pidH   .setIntegralLimit(1.0);

        // Output magnitude sanity
        pidLong.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidLat .setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidH   .setOutputLimits(-DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);
    }

    // Optional knobs
    public TrajectoryFollower setAlphaFeedforward(double kFFalphaSec){
        this.kFFAlpha = Math.max(0.0, kFFalphaSec);
        return this;
    }
    public TrajectoryFollower setCrossTrackToHeadingGain(double k){
        this.kEyToW = Math.max(0.0, k);
        return this;
    }

    public void setTrajectory(Trajectory t, double nowSec){
        this.traj = t;
        this.t0 = nowSec;
        this.lastT = nowSec;
        pidLong.reset(); pidLat.reset(); pidH.reset();
        sX.reset(0.0); sY.reset(0.0); sW.reset(0.0);
        lastElong = lastElat = lastEh = 0.0;
        // cache end state for finish checks
        if (t != null) endState = t.allStates().get(t.allStates().size()-1);
    }

    public boolean isFinished(double nowSec){
        if (traj == null) return true;

        // 1) time-based
        if ((nowSec - t0) >= traj.duration()) return true;

        // 2) end-of-path proximity (helps if robot crawls or stalls)
        Trajectory.State ref = traj.sample(nowSec - t0);
        if (endState != null) {
            boolean nearEndS = (endState.s - ref.s) <= END_S_EPS;
            Pose2d cur = poseSupplier.getPose();
            double dx = endState.pose.x - cur.x;
            double dy = endState.pose.y - cur.y;
            double posErr = Math.hypot(dx, dy);
            double hErr = normalize(endState.pose.heading - cur.heading);

            if (nearEndS && posErr < END_POS_EPS && Math.abs(hErr) < END_HEADING_EPS) {
                return true;
            }
        }
        return false;
    }

    public void cancel(){
        traj = null;
        drive.setPowers(0,0,0,0);
    }

    public void update(double nowSec){
        if (traj == null) return;
        double dt = Math.max(1e-3, nowSec - lastT);
        lastT = nowSec;

        double t = nowSec - t0;
        Trajectory.State ref = traj.sample(t);
        Pose2d cur = poseSupplier.getPose();

        // --- 1) Path frame (Frenet) basis at reference ---
        double ch_r = cos(ref.pose.heading), sh_r = sin(ref.pose.heading);
        // tangent and normal (world frame)
        double tx = ch_r,    ty = sh_r;     // t̂
        double nx = -sh_r,   ny = ch_r;     // n̂

        // --- 2) Position error in world ---
        double dx = ref.pose.x - cur.x;
        double dy = ref.pose.y - cur.y;

        // --- 3) Decompose into along-track & cross-track errors ---
        double e_long = dx*tx + dy*ty;     // projection on t̂
        double e_lat  = dx*nx + dy*ny;     // projection on n̂ (positive = left of path)

        // --- 4) Heading error (shortest) ---
        double eh = normalize(ref.pose.heading) - normalize(cur.heading);
        eh = normalize(eh);

        // --- 5) Feedforward terms ---
        // World-frame velocity feedforward along the path tangent
        double vx_world_ff = ref.v * tx;
        double vy_world_ff = ref.v * ty;
        // Angular feedforward: ω + kFF*α (optional)
        double wFF = ref.omega + kFFAlpha * ref.alpha;

        // --- 6) PD on Frenet errors: command ALONG (long) and NORMAL (lat) target velocities ---
        // We use the PIDF’s ff input as the "desired" velocity along that axis (long FF = ref.v, lat FF = 0).
        double vLongCmd = pidLong.update(0.0, -e_long, ref.v, dt);
        double vLatCmd  = pidLat .update(0.0, -e_lat,  0.0,   dt);

        // Optional cross-track→heading coupling: add a little steering into the path when off it
        double wCmd = pidH.update(0.0, -eh, wFF, dt) + kEyToW * e_lat;

        // --- 7) Build desired world velocity from Frenet components ---
        double vx_world_cmd = vLongCmd * tx + vLatCmd * nx;
        double vy_world_cmd = vLongCmd * ty + vLatCmd * ny;

        // --- 8) Rotate world→robot frame using CURRENT robot heading ---
        double ch = cos(cur.heading), sh = sin(cur.heading);
        double vxCmd =  ch*vx_world_cmd + sh*vy_world_cmd;
        double vyCmd = -sh*vx_world_cmd + ch*vy_world_cmd;

        // --- 9) Slew & clamp ---
        vxCmd = sX.filter(vxCmd, dt);
        vyCmd = sY.filter(vyCmd, dt);
        wCmd  = sW.filter(wCmd,  dt);

        // linear clamp
        double vMag = hypot(vxCmd, vyCmd);
        if (vMag > DriveConstants.MAX_VEL_IN_S) {
            double sc = DriveConstants.MAX_VEL_IN_S / vMag;
            vxCmd *= sc; vyCmd *= sc;
        }
        // angular clamp
        if (abs(wCmd) > DriveConstants.MAX_ANG_VEL_RAD_S) {
            wCmd = copySign(DriveConstants.MAX_ANG_VEL_RAD_S, wCmd);
        }

        // --- 10) Mecanum mapping ---
        double[] ws = new double[4];
        MecanumKinematics.chassisToWheels(vxCmd, vyCmd, wCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN, ws);
        double[] pw = new double[4];
        MecanumKinematics.normalizeToPowers(ws, DriveConstants.MAX_WHEEL_SPEED_IN_S, pw);

        // LF, RF, LB, RB
        drive.setPowers(pw[0], pw[1], pw[2], pw[3]);

        // (Optional) keep these for want explicit D from last errors elsewhere
        lastElong = e_long; lastElat = e_lat; lastEh = eh;
    }

    private static double normalize(double a){
        while (a<=-Math.PI) a+=2*Math.PI;
        while (a> Math.PI)  a-=2*Math.PI;
        return a;
    }
}