package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

/** Use path tangent as heading (standard "look-forward" heading). */
public class TangentHeading implements HeadingProfile {
    private final CompositePath path;
    public TangentHeading(CompositePath path){ this.path = path; }
    public double headingAt(double s){
        PathSample ps = path.sampleS(s);
        return Math.atan2(ps.tangent.y, ps.tangent.x);
    }
}
