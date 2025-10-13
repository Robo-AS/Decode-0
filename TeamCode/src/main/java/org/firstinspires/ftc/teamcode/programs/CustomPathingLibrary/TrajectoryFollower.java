package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import static java.lang.Math.*;

/** Time-parameterized trajectory follower with Frenet (tangent/normal) regulation + ω/α feedforward. */
public class TrajectoryFollower {

    public interface PoseSupplier { Pose2d getPose(); }

    private final PoseSupplier poseSupplier;
    private final MecanumDrive drive;

    private final AdvancedPIDF pidLong = new AdvancedPIDF(DriveConstants.KP_X, DriveConstants.KI_X, DriveConstants.KD_X, DriveConstants.KF_X);
    private final AdvancedPIDF pidLat  = new AdvancedPIDF(DriveConstants.KP_Y, DriveConstants.KI_Y, DriveConstants.KD_Y, DriveConstants.KF_Y);
    private final AdvancedPIDF pidH    = new AdvancedPIDF(DriveConstants.KP_H, DriveConstants.KI_H, DriveConstants.KD_H, DriveConstants.KF_H);

    private final SlewRateLimiter sX = new SlewRateLimiter(DriveConstants.MAX_DVX_IN_S2);
    private final SlewRateLimiter sY = new SlewRateLimiter(DriveConstants.MAX_DVY_IN_S2);
    private final SlewRateLimiter sW = new SlewRateLimiter(DriveConstants.MAX_DW_RAD_S2);

    private double kFFAlpha = 0.0;   // seconds
    private double kEyToW = 0.0;     // rad/(s·in)

    public Trajectory traj = null;
    private double t0 = 0.0;
    private double lastT = 0.0;

    private static final double END_S_EPS = 0.5;       // in
    private static final double END_POS_EPS = 0.75;    // in
    private static final double END_HEADING_EPS = Math.toRadians(3.0); // rad

    public Trajectory.State endState = null;


    // Terminal settle tuning (tight!)
    private static final double SETTLE_POS_TOL = 0.30; // inches (6–8mm typical)
    private static final double SETTLE_HEAD_TOL = Math.toRadians(2.0);
    private static final double SETTLE_HOLD_TIME = 0.25; // seconds inside window

    private boolean inSettle = false;
    private double settleTimer = 0.0;
    private Pose2d terminalTarget = null;


    public TrajectoryFollower(PoseSupplier supplier, MecanumDrive drive){
        this.poseSupplier = supplier;
        this.drive = drive;

        pidLong.setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);
        pidLat .setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);
        pidH   .setDerivativeFilter(DriveConstants.D_CUTOFF_HZ, DriveConstants.LOOP_HZ);

        pidLong.setIntegralLimit(2.0);
        pidLat .setIntegralLimit(2.0);
        pidH   .setIntegralLimit(1.0);

        pidLong.setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidLat .setOutputLimits(-DriveConstants.MAX_VEL_IN_S, DriveConstants.MAX_VEL_IN_S);
        pidH   .setOutputLimits(-DriveConstants.MAX_ANG_VEL_RAD_S, DriveConstants.MAX_ANG_VEL_RAD_S);
    }

    public TrajectoryFollower setAlphaFeedforward(double kFFalphaSec){
        this.kFFAlpha = Math.max(0.0, kFFalphaSec);
        return this;
    }
    public TrajectoryFollower setCrossTrackToHeadingGain(double k){
        this.kEyToW = Math.max(0.0, k);
        return this;
    }

    public void setTrajectory(Trajectory t, double nowSec, boolean alignToNearest){
        this.traj = t;
        this.lastT = nowSec;
        pidLong.reset(); pidLat.reset(); pidH.reset();
        sX.reset(0.0); sY.reset(0.0); sW.reset(0.0);
        inSettle = false; settleTimer = 0.0;
        terminalTarget = null;

        if (t != null && !t.allStates().isEmpty()){
            endState = t.allStates().get(t.allStates().size()-1);
            terminalTarget = endState.pose; // precise target for settle
            this.t0 = alignToNearest ? (nowSec - t.closestTimeTo(poseSupplier.getPose())) : nowSec;
        } else {
            endState = null;
            this.t0 = nowSec;
        }
    }

    public boolean isFinished(double nowSec){
        if (traj == null) return true;

        if ((nowSec - t0) >= traj.duration()) return true;

        if (endState != null) {
            Trajectory.State ref = traj.sample(nowSec - t0);
            boolean nearEndS = (endState.s - ref.s) <= END_S_EPS;
            Pose2d cur = poseSupplier.getPose();
            double dx = endState.pose.x - cur.x;
            double dy = endState.pose.y - cur.y;
            double posErr = Math.hypot(dx, dy);
            double hErr = normalize(endState.pose.heading - cur.heading);
            if (nearEndS && posErr < END_POS_EPS && Math.abs(hErr) < END_HEADING_EPS) return true;
        }
        return false;
    }

    public void cancel(){
        traj = null;
        drive.setPowers(0,0,0,0);
    }

    public void update(double nowSec){
        // time step
        double dt = Math.max(1e-3, nowSec - lastT);
        lastT = nowSec;

        // ===== settle mode? =====
        if (inSettle && terminalTarget != null) {
            Pose2d cur = poseSupplier.getPose();

            double dx = terminalTarget.x - cur.x;
            double dy = terminalTarget.y - cur.y;
            double eh = normalize(terminalTarget.heading - cur.heading);

            double ch = Math.cos(cur.heading), sh = Math.sin(cur.heading);
            double ex_r =  ch*dx + sh*dy;
            double ey_r = -sh*dx + ch*dy;

            double kpx = 2.0, kpy = 2.0, kph = 4.0;
            double vxCmd = clamp(kpx * ex_r, -6.0, 6.0);
            double vyCmd = clamp(kpy * ey_r, -6.0, 6.0);
            double wCmd  = clamp(kph * eh,   -Math.toRadians(120), Math.toRadians(120));

            vxCmd = sX.filter(vxCmd, dt);
            vyCmd = sY.filter(vyCmd, dt);
            wCmd  = sW.filter(wCmd,  dt);

            double[] pw = new double[4];
            MecanumKinematics.toWheelPowersPrioritized(
                    vxCmd, vyCmd, wCmd,
                    DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                    DriveConstants.MAX_WHEEL_SPEED_IN_S, pw);
            drive.setPowers(pw[0], pw[1], pw[2], pw[3]);

            boolean posOK  = Math.hypot(dx, dy) < SETTLE_POS_TOL;
            boolean headOK = Math.abs(eh) < SETTLE_HEAD_TOL;
            if (posOK && headOK) {
                settleTimer += dt;
                if (settleTimer >= SETTLE_HOLD_TIME) {
                    drive.setPowers(0,0,0,0);
                    traj = null; inSettle=false;
                }
            } else settleTimer = 0.0;
            return;
        }

        // ===== no traj =====
        if (traj == null) { drive.setPowers(0,0,0,0); return; }

        // ===== normal tracking =====
        double t = nowSec - t0;
        Trajectory.State ref = traj.sample(t);
        Pose2d cur = poseSupplier.getPose();

        // Frenet frame from PATH TANGENT (not heading)
        double tx = ref.tangent.x, ty = ref.tangent.y;
        double nrm = Math.hypot(tx,ty);
        if (nrm < 1e-6) { tx = Math.cos(ref.pose.heading); ty = Math.sin(ref.pose.heading); }
        else { tx/=nrm; ty/=nrm; }
        double nx = -ty, ny = tx;

        // position error (world)
        double dx = ref.pose.x - cur.x;
        double dy = ref.pose.y - cur.y;

        // decompose
        double e_long = dx*tx + dy*ty;
        double e_lat  = dx*nx + dy*ny;

        // heading error (shortest)
        double eh = normalize(ref.pose.heading) - normalize(cur.heading);
        eh = normalize(eh);

        // feedforward
        double wFF = ref.omega + kFFAlpha * ref.alpha;

        // PIDF (long FF=ref.v, lat FF=0)
        double vLongCmd = pidLong.update(0.0, -e_long, ref.v, dt);
        double vLatCmd  = pidLat .update(0.0, -e_lat,  0.0,   dt);
        double wCmd     = pidH   .update(0.0, -eh,     wFF,   dt) + kEyToW * e_lat;

        // world velocity from Frenet
        double vx_world_cmd = vLongCmd * tx + vLatCmd * nx;
        double vy_world_cmd = vLongCmd * ty + vLatCmd * ny;

        // world -> robot
        double ch = Math.cos(cur.heading), sh = Math.sin(cur.heading);
        double vxCmd =  ch*vx_world_cmd + sh*vy_world_cmd;
        double vyCmd = -sh*vx_world_cmd + ch*vy_world_cmd;

        // slew & clamp
        vxCmd = sX.filter(vxCmd, dt);
        vyCmd = sY.filter(vyCmd, dt);
        wCmd  = sW.filter(wCmd,  dt);

        double vMag = Math.hypot(vxCmd, vyCmd);
        if (vMag > DriveConstants.MAX_VEL_IN_S) {
            double sc = DriveConstants.MAX_VEL_IN_S / vMag;
            vxCmd *= sc; vyCmd *= sc;
        }
        if (Math.abs(wCmd) > DriveConstants.MAX_ANG_VEL_RAD_S) {
            wCmd = Math.copySign(DriveConstants.MAX_ANG_VEL_RAD_S, wCmd);
        }

        // translation-first allocator (prevents ω from stealing XY)
        double[] pw = new double[4];
        MecanumKinematics.toWheelPowersPrioritized(
                vxCmd, vyCmd, wCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN,
                DriveConstants.MAX_WHEEL_SPEED_IN_S, pw);
        drive.setPowers(pw[0], pw[1], pw[2], pw[3]);

        // enter precise settle near the end
        boolean timeDone = (t >= traj.duration() - 1e-3);
        boolean nearEndWindow = false;
        if (endState != null) {
            double ex = endState.pose.x - cur.x;
            double ey = endState.pose.y - cur.y;
            double posErr = Math.hypot(ex, ey);
            double hErr = normalize(endState.pose.heading - cur.heading);
            nearEndWindow = (posErr < 0.7) && (Math.abs(hErr) < Math.toRadians(5.0));
        }
        if (timeDone || nearEndWindow) {
            inSettle = true;
            settleTimer = 0.0;
            terminalTarget = (endState != null) ? endState.pose : ref.pose;
        }
    }

    // --- helpers & telemetry hooks ---
    public double getElapsed(double nowSec){ return nowSec - t0; }
    public Trajectory.State getCurrentRef(double nowSec){ return (traj==null)? null : traj.sample(nowSec - t0); }

    private static double normalize(double a){
        while (a<=-Math.PI) a+=2*Math.PI;
        while (a> Math.PI)  a-=2*Math.PI;
        return a;
    }

    private static double clamp(double x, double lo, double hi){
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }
}