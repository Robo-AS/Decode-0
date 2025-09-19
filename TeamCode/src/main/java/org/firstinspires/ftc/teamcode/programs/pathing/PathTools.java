package org.firstinspires.ftc.teamcode.programs.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import com.pedropathing.geometry.Pose;

public class PathTools {

    /**
     * Core: compress raw A* grid cells to corner points and convert to inches (x,y).
     * No headings here—used internally by the heading-aware methods below.
     */
    public static List<double[]> compressToCornersInches(List<int[]> cells){
        if (cells == null || cells.size() < 2) return Collections.emptyList();

        List<double[]> out = new ArrayList<double[]>();
        int[] first = cells.get(0);
        int[] dir   = stepDir(first, cells.get(1));

        // add start point (inches)
        out.add(new double[]{ CostMap.toXin(first[0]), CostMap.toYin(first[1]) });

        for (int i = 2; i < cells.size(); i++) {
            int[] cur = cells.get(i - 1);
            int[] nxt = cells.get(i);
            int[] d   = stepDir(cur, nxt);

            if (d[0] != dir[0] || d[1] != dir[1]) {
                out.add(new double[]{ CostMap.toXin(cur[0]), CostMap.toYin(cur[1]) });
                dir = d;
            }
        }

        // final goal point
        int[] last = cells.get(cells.size() - 1);
        out.add(new double[]{ CostMap.toXin(last[0]), CostMap.toYin(last[1]) });

        return out;
    }

    /**
     * Make Pedro Poses where:
     * - start pose uses headingRadStart,
     * - every MID waypoint faces the NEXT waypoint,
     * - final pose uses headingRadEnd.
     */
    public static List<Pose> toCornerPosesFacingNext(List<int[]> cells,
                                                     double headingRadStart,
                                                     double headingRadEnd) {
        List<double[]> pts = compressToCornersInches(cells);
        if (pts.isEmpty()) return Collections.emptyList();

        List<Pose> out = new ArrayList<Pose>();

        // Start pose
        double[] p0 = pts.get(0);
        out.add(new Pose(p0[0], p0[1], headingRadStart));

        // Midpoints: face the NEXT waypoint
        for (int i = 1; i < pts.size() - 1; i++) {
            double[] cur = pts.get(i);
            double[] nxt = pts.get(i + 1);
            double heading = Math.atan2(nxt[1] - cur[1], nxt[0] - cur[0]); // radians
            out.add(new Pose(cur[0], cur[1], normalizeAngle(heading)));
        }

        // End pose
        double[] pe = pts.get(pts.size() - 1);
        out.add(new Pose(pe[0], pe[1], headingRadEnd));

        return out;
    }

    /**
     * Make Pedro Poses that FACE a (possibly moving) target, e.g. Limelight-detected object.
     * Provide a Supplier that returns the target's field coords in inches: double[]{xIn, yIn}.
     * - start/end headings can be forced via params,
     * - midpoints face the current target position.
     */
    public static List<Pose> toCornerPosesFacingTarget(List<int[]> cells,
                                                       Supplier<double[]> targetInchesSupplier,
                                                       double headingRadStart,
                                                       double headingRadEnd) {
        List<double[]> pts = compressToCornersInches(cells);
        if (pts.isEmpty()) return Collections.emptyList();

        List<Pose> out = new ArrayList<Pose>();

        // Start pose
        double[] p0 = pts.get(0);
        out.add(new Pose(p0[0], p0[1], headingRadStart));

        // Midpoints: face target (each call allows a live-updating target)
        for (int i = 1; i < pts.size() - 1; i++) {
            double[] cur = pts.get(i);
            double[] tgt = (targetInchesSupplier != null) ? targetInchesSupplier.get() : null;

            double heading;
            if (tgt != null && tgt.length >= 2) {
                heading = Math.atan2(tgt[1] - cur[1], tgt[0] - cur[0]);
            } else {
                // fallback: face next waypoint if no valid target
                double[] nxt = pts.get(i + 1);
                heading = Math.atan2(nxt[1] - cur[1], nxt[0] - cur[0]);
            }
            out.add(new Pose(cur[0], cur[1], normalizeAngle(heading)));
        }

        // End pose
        double[] pe = pts.get(pts.size() - 1);
        out.add(new Pose(pe[0], pe[1], headingRadEnd));

        return out;
    }

    // Direction between two adjacent cells, each component ∈ {-1, 0, +1}
    private static int[] stepDir(int[] a, int[] b){
        return new int[]{ Integer.signum(b[0] - a[0]), Integer.signum(b[1] - a[1]) };
    }

    private static double normalizeAngle(double rad){
        // normalize to (-PI, PI]
        while (rad <= -Math.PI) rad += 2*Math.PI;
        while (rad >   Math.PI) rad -= 2*Math.PI;
        return rad;
    }
}
