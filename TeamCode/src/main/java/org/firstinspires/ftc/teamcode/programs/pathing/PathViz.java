package org.firstinspires.ftc.teamcode.programs.pathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;   // <-- important import

import java.util.List;

public class PathViz {
    private final FtcDashboard dash = FtcDashboard.getInstance();

    public void draw(CostMap map, int sx, int sy, int gx, int gy, List<int[]> path){
        TelemetryPacket pkt = new TelemetryPacket();
        Canvas c = pkt.fieldOverlay();   // <-- no 'var'

        // blocked cells (red)
        for (int y = 0; y < Field.NY; y++) {
            for (int x = 0; x < Field.NX; x++) {
                if (!Double.isFinite(map.cost[y][x])) {
                    double cx = CostMap.toXcm(x), cy = CostMap.toYcm(y);
                    c.setFill("rgba(200,0,0,0.45)")
                            .fillRect(cx - Field.CELL/2, cy - Field.CELL/2, Field.CELL, Field.CELL);
                }
            }
        }

        // start (green) & goal (blue)
        c.setFill("green").fillCircle(CostMap.toXcm(sx), CostMap.toYcm(sy), 2);
        c.setFill("blue").fillCircle(CostMap.toXcm(gx), CostMap.toYcm(gy), 2);

        // path (yellow)
        if (path != null && !path.isEmpty()){
            c.setStroke("yellow").setStrokeWidth(2);
            for (int i = 1; i < path.size(); i++) {
                double x0 = CostMap.toXcm(path.get(i-1)[0]), y0 = CostMap.toYcm(path.get(i-1)[1]);
                double x1 = CostMap.toXcm(path.get(i)[0]),   y1 = CostMap.toYcm(path.get(i)[1]);
                c.strokeLine(x0, y0, x1, y1);
            }
        }

        dash.sendTelemetryPacket(pkt);
    }
}
