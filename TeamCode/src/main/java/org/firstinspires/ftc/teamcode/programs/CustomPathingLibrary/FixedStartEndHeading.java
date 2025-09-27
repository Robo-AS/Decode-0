package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/** Linearly interpolate heading between fixed start and end headings along s. */
public class FixedStartEndHeading implements HeadingProfile {
    private final double startH, endH, totalS;
    public FixedStartEndHeading(double startH, double endH, double totalS){
        this.startH=startH; this.endH=endH; this.totalS=Math.max(1e-9, totalS);
    }
    public double headingAt(double s){
        double t = Math.max(0.0, Math.min(1.0, s/totalS));
        double dh = normalize(endH) - normalize(startH);
        dh = normalize(dh);
        return normalize(startH + dh*t);
    }
    private static double normalize(double a){
        while (a<=-Math.PI) a+=2*Math.PI;
        while (a> Math.PI) a-=2*Math.PI;
        return a;
    }
}
