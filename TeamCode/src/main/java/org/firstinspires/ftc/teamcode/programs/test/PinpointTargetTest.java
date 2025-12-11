package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "Pinpoint Turret Angle Test", group = "Test")
public class PinpointTargetTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static boolean targetRedGoal = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.initializeHardware(hardwareMap);
        robot.initialize();
    }

    @Override
    public void run() {
        robot.pinpoint.update();

        double x = robot.pinpoint.getPosX(DistanceUnit.INCH);
        double y = robot.pinpoint.getPosY(DistanceUnit.INCH);
        double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);

        // FIELD TARGET
        double goalX = targetRedGoal ? 144 : 0;
        double goalY = 144;

        // 1. Field angle toward target
        double fieldAngleDeg =
                Math.toDegrees(Math.atan2(goalY - y, goalX - x));

        // 2. Convert to robot-local angle (Pinpoint frame)
        double localAngle = fieldAngleDeg - Math.toDegrees(heading);

        // 3. Convert Pinpoint left/right to YOUR turret's left/right
        double turretAngle = -localAngle;

        // 4. Normalize
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

        telemetry.addData("Robot X (in)", x);
        telemetry.addData("Robot Y (in)", y);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Turret Target Angle (deg)", turretAngle);
        telemetry.update();
    }
}
