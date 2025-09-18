package org.firstinspires.ftc.teamcode.programs.pathing;

public final class Field {
    // 12ft field ≈ 366 cm; origin at field center
    public static final double HALF = 183.0;     // cm
    public static final double CELL = 5.0;       // cm per grid cell (tune 5–10)
    public static final int NX = (int)Math.round((2*HALF)/CELL); // grid width
    public static final int NY = (int)Math.round((2*HALF)/CELL); // grid height

    public static final double MAX_SPEED_CM_S = 60.0; // crude avg robot speed; tune
    private Field() {}
}
