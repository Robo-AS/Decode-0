package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import java.util.ArrayList;
import java.util.List;

/** A list of ParametricPath segments sampled and stitched by arc length. */
public class CompositePath {
    private static class SampleU {
        final double s;   // cumulative arc length (in)
        final int segIdx;
        final double u;
        final Vector2d pos, dp, ddp; // dp, ddp are w.r.t. parameter u
        SampleU(double s,int segIdx,double u,Vector2d pos,Vector2d dp,Vector2d ddp){
            this.s=s; this.segIdx=segIdx; this.u=u; this.pos=pos; this.dp=dp; this.ddp=ddp;
        }
    }

    private final List<ParametricPath> segments = new ArrayList<>();
    private final List<SampleU> table = new ArrayList<>();
    private double totalLength = 0.0;

    public void add(ParametricPath seg){ segments.add(seg); }

    /** Legacy: keep for compatibility. Uses a rough ds from samplesPerSegment. */
    public void build(int samplesPerSegment){
        double ds = 0.5;
        if (samplesPerSegment > 0) {
            // crude mapping: ~segmentLen/sps; will be overridden per segment anyway
            ds = Math.max(0.1, Math.min(2.0, 200.0/Math.max(10, samplesPerSegment)));
        }
        buildByDs(ds);
    }

    /** Preferred: sample each segment at ~uniform ds along arc length. */
    public void buildByDs(double ds){
        table.clear();
        totalLength = 0.0;
        ds = Math.max(1e-3, ds);

        for (int i = 0; i < segments.size(); i++){
            ParametricPath seg = segments.get(i);

            double L = Math.max(1e-6, seg.length());
            int N = Math.max(4, (int)Math.ceil(L/ds));

            Vector2d prev = seg.p(0);
            Vector2d prevDp = seg.dp(0);
            Vector2d prevDdp = seg.ddp(0);
            table.add(new SampleU(totalLength, i, 0.0, prev, prevDp, prevDdp));

            for (int k = 1; k <= N; k++){
                double u = (double)k / N;
                Vector2d cur  = seg.p(u);
                Vector2d dp   = seg.dp(u);
                Vector2d ddp  = seg.ddp(u);

                totalLength += cur.minus(prev).norm();

                table.add(new SampleU(totalLength, i, u, cur, dp, ddp));
                prev = cur; prevDp = dp; prevDdp = ddp;
            }
        }
    }

    public void clear(){ segments.clear(); table.clear(); totalLength = 0.0; }
    public double length(){ return totalLength; }

    /** Return sample at arc-length s (clamped) with tangent & curvature. */
    public PathSample sampleS(double s){
        if (table.isEmpty()) throw new IllegalStateException("CompositePath not built");
        if (s <= 0) return toPathSample(table.get(0));
        if (s >= totalLength) return toPathSample(table.get(table.size()-1));

        int lo=0, hi=table.size()-1;
        while (hi - lo > 1){
            int mid = (lo+hi)>>>1;
            if (table.get(mid).s < s) lo=mid; else hi=mid;
        }

        SampleU a = table.get(lo), b = table.get(hi);
        double span = Math.max(1e-9, b.s - a.s);
        double t = (s - a.s)/span;

        Vector2d pos = new Vector2d(
                a.pos.x + (b.pos.x - a.pos.x)*t,
                a.pos.y + (b.pos.y - a.pos.y)*t
        );
        Vector2d dp = new Vector2d(
                a.dp.x + (b.dp.x - a.dp.x)*t,
                a.dp.y + (b.dp.y - a.dp.y)*t
        );
        Vector2d ddp = new Vector2d(
                a.ddp.x + (b.ddp.x - a.ddp.x)*t,
                a.ddp.y + (b.ddp.y - a.ddp.y)*t
        );

        // curvature kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        double x1 = dp.x, y1 = dp.y, x2 = ddp.x, y2 = ddp.y;
        double denom = Math.pow(Math.max(1e-9, x1*x1 + y1*y1), 1.5);
        double kappa = (x1*y2 - y1*x2) / denom;

        Vector2d tan = dp.normalized();
        return new PathSample(s, pos, tan, kappa);
    }

    private PathSample toPathSample(SampleU su){
        Vector2d dp = su.dp;
        double x1 = dp.x, y1 = dp.y, x2 = su.ddp.x, y2 = su.ddp.y;
        double denom = Math.pow(Math.max(1e-9, x1*x1 + y1*y1), 1.5);
        double kappa = (x1*y2 - y1*x2) / denom;
        return new PathSample(su.s, su.pos, dp.normalized(), kappa);
    }
}