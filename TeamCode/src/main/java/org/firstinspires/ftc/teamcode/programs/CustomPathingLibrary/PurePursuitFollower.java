package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import static java.lang.Math.*;

/**
 * Pure Pursuit path follower for mecanum.
 * - Pose source is injected via PoseSupplier (so you can use Pinpoint, 2-wheel+IMU, etc.).
 * - Gains, lookahead and velocity limits are all configurable at runtime.
 * - Produces chassis commands (vx, vy, omega) and maps to wheel powers via MecanumKinematics + MecanumDrive.
 *
 * Units:
 *   x,y in inches; heading in radians; velocities in in/s and rad/s.
 */
public class PurePursuitFollower {

    // --- Pose supplier (localizer) ---
    public interface PoseSupplier { Pose2d getPose(); }

    private final PoseSupplier poseSupplier;
    private final MecanumDrive drive;

    // --- Tunables (with sensible defaults from DriveConstants) ---
    // linear and angular PD gains (I is rarely helpful for PP)
    private double kP_lin =  DriveConstants.KP_X;
    private double kD_lin =  DriveConstants.KD_X;
    private double kP_ang =  DriveConstants.KP_H;
    private double kD_ang =  DriveConstants.KD_H;

    // lookahead distance range (in)
    private double lookaheadMin = 6.0;
    private double lookaheadMax = 18.0;

    // velocity limits (chassis)
    private double maxVel      = DriveConstants.MAX_VEL_IN_S;
    private double maxOmega    = DriveConstants.MAX_ANG_VEL_RAD_S;

    // optional slew limits
    private final SlewRateLimiter slewVx = new SlewRateLimiter(DriveConstants.MAX_DVX_IN_S2);
    private final SlewRateLimiter slewVy = new SlewRateLimiter(DriveConstants.MAX_DVY_IN_S2);
    private final SlewRateLimiter slewW  = new SlewRateLimiter(DriveConstants.MAX_DW_RAD_S2);

    // state
    private PathSample[] path = null;  // discretized path points (x,y,headingAlongPath)
    private int lastNearestIdx = 0;
    private double lastEx = 0, lastEy = 0, lastEh = 0;

    // --- Constructors ---
    public PurePursuitFollower(PoseSupplier supplier, MecanumDrive drive){
        this.poseSupplier = supplier;
        this.drive = drive;
    }

    /** Compatibility/”kitchen-sink” ctor if your opmode passes many knobs up front. */
    public PurePursuitFollower(
            PoseSupplier supplier, MecanumDrive drive,
            double kP_lin, double kD_lin, double kP_ang, double kD_ang,
            double lookaheadMin, double lookaheadMax,
            double maxVel, double maxOmega
    ){
        this(supplier, drive);
        setGains(kP_lin, kD_lin, kP_ang, kD_ang);
        setLookahead(lookaheadMin, lookaheadMax);
        setVelocityLimits(maxVel, maxOmega);
    }

    // --- Config setters (chainable) ---
    public PurePursuitFollower setGains(double kP_lin, double kD_lin, double kP_ang, double kD_ang){
        this.kP_lin = kP_lin; this.kD_lin = kD_lin;
        this.kP_ang = kP_ang; this.kD_ang = kD_ang;
        return this;
    }
    public PurePursuitFollower setLookahead(double minIn, double maxIn){
        this.lookaheadMin = Math.max(1e-3, minIn);
        this.lookaheadMax = Math.max(lookaheadMin, maxIn);
        return this;
    }
    public PurePursuitFollower setVelocityLimits(double maxVelInPerSec, double maxOmegaRadPerSec){
        this.maxVel   = Math.max(1e-3, maxVelInPerSec);
        this.maxOmega = Math.max(1e-3, maxOmegaRadPerSec);
        return this;
    }

    /** Provide the discretized path you want to chase (dense enough for smoothness). */
    public void setPath(PathSample[] samples){
        this.path = samples;
        this.lastNearestIdx = 0;
        this.lastEx = this.lastEy = this.lastEh = 0;
        slewVx.reset(0.0); slewVy.reset(0.0); slewW.reset(0.0);
    }

    public boolean hasPath(){ return path != null && path.length >= 2; }

    /** Call this every loop with dt (s). It computes commands and sends wheel powers to drive. */
    public void update(double dt){
        if (!hasPath()) { drive.setPowers(0,0,0,0); return; }
        Pose2d pose = poseSupplier.getPose();

        // 1) find nearest segment point (progressively from last)
        int nearest = findNearestPointIndex(pose.x, pose.y, lastNearestIdx);
        lastNearestIdx = nearest;

        // 2) choose lookahead distance as function of speed/error (simple adaptive)
        double la = adaptiveLookahead(pose, nearest);
        // 3) find lookahead point along path
        PathSample target = findLookaheadPoint(nearest, la);

        // 4) compute geometric pursuit command
        // vector from robot -> lookahead (field)
        double dxF = target.x - pose.x;
        double dyF = target.y - pose.y;

        // transform to robot frame
        double sh = sin(pose.heading), ch = cos(pose.heading);
        double dxR =  ch*dxF + sh*dyF;
        double dyR = -sh*dxF + ch*dyF;

        // “go there” linear command (PD on robot-frame error)
        double ex = dxR, ey = dyR;
        double dex = (ex - lastEx)/max(1e-3, dt);
        double dey = (ey - lastEy)/max(1e-3, dt);
        lastEx = ex; lastEy = ey;

        double vxCmd = kP_lin*ex + kD_lin*dex;
        double vyCmd = kP_lin*ey + kD_lin*dey;

        // face path tangent (or target heading)
        double desiredHeading = target.heading; // tangent heading stored in sample
        double eh = normalize(desiredHeading - pose.heading);
        double deh = (eh - lastEh)/max(1e-3, dt);
        lastEh = eh;

        double wCmd = kP_ang*eh + kD_ang*deh;

        // 5) limit/slew
        // clamp linear vector magnitude
        double vMag = hypot(vxCmd, vyCmd);
        if (vMag > maxVel) {
            double sc = maxVel / vMag;
            vxCmd *= sc; vyCmd *= sc;
        }
        if (abs(wCmd) > maxOmega) wCmd = copySign(maxOmega, wCmd);

        vxCmd = slewVx.filter(vxCmd, dt);
        vyCmd = slewVy.filter(vyCmd, dt);
        wCmd  = slewW.filter(wCmd,  dt);

        // 6) mecanum mapping -> wheel powers
        double[] wheelSpeeds = new double[4];
        MecanumKinematics.chassisToWheels(vxCmd, vyCmd, wCmd,
                DriveConstants.TRACKWIDTH_IN, DriveConstants.WHEELBASE_IN, wheelSpeeds);

        double[] wheelPowers = new double[4];
        MecanumKinematics.normalizeToPowers(wheelSpeeds, DriveConstants.MAX_WHEEL_SPEED_IN_S, wheelPowers);

        // IMPORTANT: order is LF, RF, LB, RB (matches your MecanumDrive.setPowers)
        drive.setPowers(wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]);
    }

    // ---------- helpers ----------

    private int findNearestPointIndex(double x, double y, int startIdx){
        int best = startIdx;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = startIdx; i < path.length; i++){
            double dx = x - path[i].x, dy = y - path[i].y;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestD2){ bestD2 = d2; best = i; }
        }
        return best;
    }

    private double adaptiveLookahead(Pose2d pose, int nearestIdx){
        // Simple heuristic: more error → larger LA; near end → shrink LA
        double errPos = distance(pose.x, pose.y, path[nearestIdx].x, path[nearestIdx].y);
        double u = clamp(errPos / 24.0, 0.0, 1.0); // scale 0..1 over ~24 inches of error
        return lerp(lookaheadMin, lookaheadMax, u);
    }

    private PathSample findLookaheadPoint(int fromIdx, double lookaheadIn){
        double remaining = lookaheadIn;
        int i = fromIdx;
        while (i < path.length - 1){
            double seg = distance(path[i].x, path[i].y, path[i+1].x, path[i+1].y);
            if (seg >= remaining){
                double t = remaining / Math.max(1e-6, seg);
                double x = lerp(path[i].x, path[i+1].x, t);
                double y = lerp(path[i].y, path[i+1].y, t);
                double h = slerpAngle(path[i].heading, path[i+1].heading, t);
                return new PathSample(x,y,h);
            }
            remaining -= seg;
            i++;
        }
        // end of path
        return path[path.length - 1];
    }

    private static double distance(double x0, double y0, double x1, double y1){
        return hypot(x1 - x0, y1 - y0);
    }
    private static double lerp(double a, double b, double t){ return a + (b - a)*t; }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }

    private static double slerpAngle(double a, double b, double t){
        double d = normalize(b - a);
        return normalize(a + d * t);
    }
    private static double normalize(double a){
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }

    // ---- small path sample struct ----
    public static class PathSample {
        public final double x, y, heading;
        public PathSample(double x, double y, double heading){
            this.x = x; this.y = y; this.heading = heading;
        }
    }
}