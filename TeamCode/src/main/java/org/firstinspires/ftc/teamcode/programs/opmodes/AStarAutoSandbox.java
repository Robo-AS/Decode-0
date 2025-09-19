package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.pathing.*;
import org.firstinspires.ftc.teamcode.programs.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import java.util.*;

/**
 * A* cycle auto: pick 3 artifacts -> score -> repeat.
 * - All units are inches, field-centered (±72), matching Pedro.
 * - A* runs on a fresh cost map every loop (walls only by default).
 * - Artifacts are GOALS (not blocked) so the robot drives TO them.
 * - Limelight updates artifact positions live (smoothed).
 * - Path + artifacts are drawn on FTC Dashboard.
 *
 * NOTE: This version "simulates" motion by warping robotX/robotY to goal on plan completion.
 * Hook your Pedro follower where marked to actually drive.
 */
@Autonomous(name="A* Cycle: Pick 3 -> Score -> Repeat")
public class AStarAutoSandbox extends LinearOpMode {

    // === CONFIG ===
    private static final boolean RIGHT_SIDE = false;

    // how many pieces we can hold
    private static final int CAPACITY = 3;
    // arrival threshold (inches) to consider we reached a goal
    private static final double ARRIVE_DIST_IN = 5.0;
    // smoothing for Limelight updates (0..1)
    private static final double VISION_ALPHA = 0.45;

    // scoring location (tune; field-centered inches; mirror X if RIGHT_SIDE)
    private static Pose scorePose() {
        double x = 52.0, y = 48.0, h = Math.toRadians(0);
        if (RIGHT_SIDE) x = -x;
        return new Pose(x, y, h);
    }

    // which color to collect
    private enum ColorPref { PURPLE_ONLY, GREEN_ONLY, EITHER }
    private ColorPref colorPref = ColorPref.EITHER;

    // simple state machine
    private enum Phase { COLLECT, SCORE }
    private Phase phase = Phase.COLLECT;

    // live cargo count
    private int inventory = 0;

    // ====== ARTIFACT TRACKING ======
    private static class TrackedArtifact {
        final String id;       // label (P1, G2, etc.)
        final String type;     // "purple" or "green"
        double xIn, yIn;       // inches, field-centered
        boolean seen = false;  // was updated from vision
        boolean taken = false; // already collected

        TrackedArtifact(String id, String type, double xIn, double yIn) {
            this.id = id; this.type = type; this.xIn = xIn; this.yIn = yIn;
        }
        void updateFromVision(double vx, double vy, double alpha) {
            xIn = xIn + alpha * (vx - xIn);
            yIn = yIn + alpha * (vy - yIn);
            seen = true;
        }
    }

    // maps of current known artifacts
    private final Map<String, TrackedArtifact> purpleMap = new HashMap<>();
    private final Map<String, TrackedArtifact> greenMap  = new HashMap<>();

    private Collection<double[]> purplePoints(){
        List<double[]> out = new ArrayList<>();
        for (TrackedArtifact t : purpleMap.values()) if (!t.taken) out.add(new double[]{t.xIn, t.yIn});
        return out;
    }
    private Collection<double[]> greenPoints(){
        List<double[]> out = new ArrayList<>();
        for (TrackedArtifact t : greenMap.values()) if (!t.taken) out.add(new double[]{t.xIn, t.yIn});
        return out;
    }

    // ====== ROBOT + PATHING ======
    private final Robot robot = Robot.getInstance();
    private PathViz viz;

    // rough robot position estimate (replace with odometry when you hook Pedro)
    private double robotX = -40, robotY = -40; // start pose (inches, centered)
    // current goal we’re heading to
    private Pose currentGoal = null;

    // placeholder: convert Limelight artifact -> field-centered inches
    // (replace with a.toFieldInches() if you prefer your Limelight method)
    private double[] toFieldInchesCentered(Limelight.Artifact a) {
        final boolean normalized = true; // set false if your Python outputs pixels
        double u = a.cx, v = a.cy;
        if (!normalized) {
            final double IMG_W = 640.0, IMG_H = 480.0;
            u = (a.cx - IMG_W/2.0) / (IMG_W/2.0);
            v = (a.cy - IMG_H/2.0) / (IMG_H/2.0);
        }
        // map normalized -> field inches (tune spans until dots look reasonable)
        final double X_SPAN = 48.0, Y_SPAN = 36.0;
        return new double[]{ u * X_SPAN, v * Y_SPAN };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeHardware(hardwareMap);
        robot.initialize();
        viz = new PathViz();

        // ---- Seed PRIORS (measure/tune; field-centered inches) ----
        // Add at least 3 in the color you're targeting for the cycle.
        purpleMap.put("P1", new TrackedArtifact("P1", "purple", -30.0, -10.0));
        purpleMap.put("P2", new TrackedArtifact("P2", "purple", -24.0, -12.0));
        purpleMap.put("P3", new TrackedArtifact("P3", "purple", -18.0,  -8.0));

        greenMap.put("G1",  new TrackedArtifact("G1", "green",   30.0, -10.0));
        greenMap.put("G2",  new TrackedArtifact("G2", "green",   24.0, -12.0));
        greenMap.put("G3",  new TrackedArtifact("G3", "green",   18.0,  -8.0));

        if (RIGHT_SIDE) { // mirror X if you flip alliance wall
            for (TrackedArtifact t : purpleMap.values()) t.xIn = -t.xIn;
            for (TrackedArtifact t : greenMap.values())  t.xIn = -t.xIn;
        }

        // choose an initial target so pathing has a goal pre-start
        currentGoal = chooseNextPickupGoal(robotX, robotY);

        telemetry.addLine("A* Cycle ready — open FTC Dashboard.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // === 1) Vision update: smooth-correct artifact positions ===
            Limelight ll = robot.getInstanceLimelight();
            ll.loop();
            if (!ll.artifactList.isEmpty()) {
                for (Limelight.Artifact a : ll.artifactList) {
                    double[] pos = toFieldInchesCentered(a); // or a.toFieldInches()
                    Map<String, TrackedArtifact> m = a.type.equals("purple") ? purpleMap : greenMap;

                    // nearest-of-that-color update
                    TrackedArtifact best = null; double bestD2 = Double.POSITIVE_INFINITY;
                    for (TrackedArtifact t : m.values()) {
                        if (t.taken) continue;
                        double dx = pos[0] - t.xIn, dy = pos[1] - t.yIn;
                        double d2 = dx*dx + dy*dy;
                        if (d2 < bestD2) { bestD2 = d2; best = t; }
                    }
                    if (best != null) best.updateFromVision(pos[0], pos[1], VISION_ALPHA);
                }
            }

            // === 2) State machine ===
            switch (phase) {
                case COLLECT: {
                    if (inventory < CAPACITY) {
                        // ensure we have a goal; if not, pick the nearest untaken target
                        if (currentGoal == null) {
                            currentGoal = chooseNextPickupGoal(robotX, robotY);
                        }
                        // plan & (in real robot) drive to currentGoal
                        if (currentGoal != null) {
                            List<Pose> wps = planFromRobotTo(currentGoal.getX(), currentGoal.getY());
                            // TODO: SEND waypoints to Pedro follower here
                            // follower.follow(pathFrom(wps));
                            // while (opModeIsActive() && follower.isBusy()) idle();

                            // First iteration: simulate arrival
                            if (distance(robotX, robotY, currentGoal.getX(), currentGoal.getY()) <= ARRIVE_DIST_IN) {
                                // startIntake(); wait sensor; stopIntake();  (hook real code)
                                // mark nearest target at that spot as collected
                                markNearestAsTaken(currentGoal.getX(), currentGoal.getY(), desiredColorString());
                                inventory = Math.min(CAPACITY, inventory + 1);
                                // update robot estimate to that spot (until odometry available)
                                robotX = currentGoal.getX();
                                robotY = currentGoal.getY();

                                if (inventory >= CAPACITY) {
                                    currentGoal = scorePose();
                                    phase = Phase.SCORE;
                                } else {
                                    currentGoal = chooseNextPickupGoal(robotX, robotY);
                                }
                            } else {
                                // If you integrate the follower, update robotX/robotY from odometry each loop.
                            }
                        } else {
                            // No targets known → fall back to scoring or wait
                            currentGoal = scorePose();
                            phase = Phase.SCORE;
                        }
                    } else {
                        // bag filled asynchronously → go score
                        currentGoal = scorePose();
                        phase = Phase.SCORE;
                    }
                    break;
                }

                case SCORE: {
                    // go to scoring pose
                    if (currentGoal == null) currentGoal = scorePose();
                    List<Pose> wps = planFromRobotTo(currentGoal.getX(), currentGoal.getY());
                    // TODO: send to Pedro follower & drive

                    // simulate arrival
                    if (distance(robotX, robotY, currentGoal.getX(), currentGoal.getY()) <= ARRIVE_DIST_IN) {
                        // performScoring(); (hook real mechanism)
                        inventory = 0;
                        // after scoring, pick next
                        robotX = currentGoal.getX();
                        robotY = currentGoal.getY();
                        currentGoal = chooseNextPickupGoal(robotX, robotY);
                        phase = Phase.COLLECT;
                    }
                    break;
                }
            }

            // === 3) Build a fresh map for viz (STATIC walls only) ===
            CostMap map = buildWallsOnlyMap();  // artifacts are targets, not obstacles

            // For the picture/path line: plan from robot -> (currentGoal or score)
            int sx = CostMap.toGX(robotX), sy = CostMap.toGY(robotY);
            int gx = (currentGoal != null) ? CostMap.toGX(currentGoal.getX()) : sx;
            int gy = (currentGoal != null) ? CostMap.toGY(currentGoal.getY()) : sy;
            List<int[]> cells = AStar.plan(map, sx, sy, gx, gy);

            // === 4) Draw ===
            viz.draw(map, sx, sy, gx, gy, cells, purplePoints(), greenPoints());

            telemetry.addData("Phase", phase);
            telemetry.addData("Inventory", "%d / %d", inventory, CAPACITY);
            telemetry.addData("Robot", "(%.1f, %.1f)", robotX, robotY);
            if (currentGoal != null)
                telemetry.addData("Goal", "(%.1f, %.1f, %.0f°)", currentGoal.getX(), currentGoal.getY(), Math.toDegrees(currentGoal.getHeading()));
            telemetry.update();

            sleep(40);
        }
    }

    // ====== helpers ======

    private CostMap buildWallsOnlyMap() {
        CostMap map = new CostMap();
        double margin = 4;
        map.blockRectIn(0,  Field.HALF - margin/2, 2*Field.HALF, margin); // top
        map.blockRectIn(0, -Field.HALF + margin/2, 2*Field.HALF, margin); // bottom
        map.blockRectIn( Field.HALF - margin/2, 0, margin, 2*Field.HALF); // right
        map.blockRectIn(-Field.HALF + margin/2, 0, margin, 2*Field.HALF); // left
        map.inflate(4);
        return map;
    }

    private List<Pose> planFromRobotTo(double tx, double ty) {
        // fresh map each plan for consistency
        CostMap map = buildWallsOnlyMap();
        int sx = CostMap.toGX(robotX), sy = CostMap.toGY(robotY);
        int gx = CostMap.toGX(tx),     gy = CostMap.toGY(ty);
        List<int[]> cells = AStar.plan(map, sx, sy, gx, gy);
        return PathTools.toCornerPosesFacingNext(cells,
                Math.toRadians(180), // start heading (tune)
                0.0);                // end heading (tune per task)
    }

    private Pose chooseNextPickupGoal(double fromX, double fromY) {
        List<TrackedArtifact> candidates = new ArrayList<>();
        switch (colorPref) {
            case PURPLE_ONLY: candidates.addAll(purpleMap.values()); break;
            case GREEN_ONLY:  candidates.addAll(greenMap.values()); break;
            case EITHER:      candidates.addAll(purpleMap.values()); candidates.addAll(greenMap.values()); break;
        }
        TrackedArtifact best = null; double bestD2 = Double.POSITIVE_INFINITY;
        for (TrackedArtifact t : candidates) {
            if (t.taken) continue;
            double dx = t.xIn - fromX, dy = t.yIn - fromY;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestD2) { bestD2 = d2; best = t; }
        }
        return (best == null) ? null : new Pose(best.xIn, best.yIn, 0.0);
    }

    private void markNearestAsTaken(double atX, double atY, String color) {
        Map<String, TrackedArtifact> m = color.equals("purple") ? purpleMap : greenMap;
        TrackedArtifact best = null; double bestD2 = Double.POSITIVE_INFINITY;
        for (TrackedArtifact t : m.values()) {
            if (t.taken) continue;
            double dx = t.xIn - atX, dy = t.yIn - atY;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestD2) { bestD2 = d2; best = t; }
        }
        if (best != null) best.taken = true;
    }

    private String desiredColorString() {
        if (colorPref == ColorPref.PURPLE_ONLY) {
            return "purple";
        } else if (colorPref == ColorPref.GREEN_ONLY) {
            return "green";
        } else {
            // EITHER: default preference; change to "green" if you want the other order
            return "purple";
        }
    }

    private static double distance(double x0, double y0, double x1, double y1) {
        return Math.hypot(x1 - x0, y1 - y0);
    }

    // TODO: real mechanisms when integrating with hardware:
    // private void startIntake() { }
    // private void stopIntake() { }
    // private void performScoring() { }
}
