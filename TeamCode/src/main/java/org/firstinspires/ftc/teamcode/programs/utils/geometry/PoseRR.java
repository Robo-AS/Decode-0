package org.firstinspires.ftc.teamcode.programs.utils.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Locale;

public class PoseRR extends Point {

    public double heading;

    public PoseRR(double x, double y, double heading) {
        super(x, y);
        this.heading = AngleUnit.normalizeRadians(heading);
    }

    public PoseRR(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public PoseRR(Vector2D vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public PoseRR() {
        this(0, 0, 0);
    }

    public PoseRR(AprilTagPoseFtc ftcPose) {
        this.heading = Math.toRadians(-ftcPose.yaw);
        this.x = ftcPose.x * Math.cos(heading) - ftcPose.y * Math.sin(heading);
        this.y = ftcPose.x * Math.sin(heading) + ftcPose.y * Math.cos(heading);
    }

    public void set(PoseRR other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public PoseRR add(PoseRR other) {
        return new PoseRR(x + other.x, y + other.y, heading + other.heading);
    }

    public PoseRR subtract(PoseRR other) {
        return new PoseRR(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public PoseRR divide(PoseRR other) {
        return new PoseRR(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public PoseRR scale(double scalar){
        return new PoseRR(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public PoseRR subt(PoseRR other) {
        return new PoseRR(x - other.x, y - other.y, heading - other.heading);
    }

    public Vector2D toVec2D() {
        return new Vector2D(x, y);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.3f", x, y, heading);
    }
}