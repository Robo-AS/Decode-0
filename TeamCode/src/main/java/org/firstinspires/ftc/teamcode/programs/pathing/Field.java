package org.firstinspires.ftc.teamcode.programs.pathing;

public final class Field {
    // 12ft field = 144 inches; origin at field center
    public static final double HALF = 72.0;      // inches (half field length)
    public static final double CELL = 2.0;       // inches per grid cell (tune 2–4)
    public static final int NX = (int)Math.round((2*HALF)/CELL); // grid width
    public static final int NY = (int)Math.round((2*HALF)/CELL); // grid height

    public static final double MAX_SPEED_IN_S = 24.0; // avg speed in inches/sec (tune)
    private Field() {}
}
