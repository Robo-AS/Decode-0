package org.firstinspires.ftc.teamcode.programs.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.pathing.*;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Limelight;

import java.util.List;

@Autonomous(name="A* Sandbox (viz only)")
public class AStarSandboxAuto extends LinearOpMode {
    private final Robot robot = Robot.getInstance();

    // flip this when you switch alliance side (mirror in X)
    private static final boolean RIGHT_SIDE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        // --- Build cost map ---
        CostMap map = new CostMap();

        // 1) Block field perimeter (keep ~10cm margin inside walls)
        double margin = 10;
        map.blockRectCm(0,  Field.HALF - margin/2, 2*Field.HALF, margin); // top
        map.blockRectCm(0, -Field.HALF + margin/2, 2*Field.HALF, margin); // bottom
        map.blockRectCm( Field.HALF - margin/2, 0, margin, 2*Field.HALF); // right
        map.blockRectCm(-Field.HALF + margin/2, 0, margin, 2*Field.HALF); // left

        // 2) Inflate by robot half-width ~ 20cm/Cell ≈ 4 cells (tune to robot)
        map.inflate(4);

        // --- Choose a start & goal (in cm, field-centered) ---
        // Start near the bottom-left quadrant in example
        double startX = -120, startY = -120;

        // Goal somewhere up/right (mirror X if RIGHT_SIDE):
        double goalX  = (RIGHT_SIDE ? -1 : 1) * 120;
        double goalY  =  80;

        int sx = CostMap.toGX(startX), sy = CostMap.toGY(startY);
        int gx = CostMap.toGX(goalX),  gy = CostMap.toGY(goalY);

        PathViz viz = new PathViz();
        List<int[]> pathCells = AStar.plan(map, sx,sy,gx,gy);

        telemetry.addLine("A* Sandbox ready. Open FTC Dashboard to see the field overlay.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Main loop: (1) update limelight info, (2) re-plan if needed, (3) draw
        while (opModeIsActive()) {
            // Update Limelight (optional now; will use it next step)
            Limelight ll = robot.getInstanceLimelight();
            ll.loop();

            // QUICK DEMO: If limelight sees any artifact, block a small area in front of the start (just to see dynamic change)
            // (Next step --> convert camera detections to real field coords.)
            if (!ll.artifactList.isEmpty()) {
                map.blockRectCm(0, 0, 40, 40); // block the central 40x40 cm region
            }

            // Re-plan (cheap at this grid size)
            pathCells = AStar.plan(map, sx,sy,gx,gy);

            viz.draw(map, sx,sy, gx,gy, pathCells);

            telemetry.addData("Path cells", pathCells.size());
            telemetry.update();
            sleep(40);
        }
    }
}
