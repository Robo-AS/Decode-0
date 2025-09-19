package org.firstinspires.ftc.teamcode.programs.pathing;

import java.util.*;

public class AStar {
    private static final int[][] DIRS = {
            {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
    };

    private static final class GridNode {
        final int x, y;
        GridNode(int x, int y){ this.x = x; this.y = y; }
    }

    public static List<int[]> plan(CostMap map, int sx,int sy, int gx,int gy){
        int nX = Field.NX, nY = Field.NY;
        double[][] g = new double[nY][nX];
        for (double[] row : g) Arrays.fill(row, Double.POSITIVE_INFINITY);

        GridNode[][] parent = new GridNode[nY][nX];
        boolean[][] closed = new boolean[nY][nX];

        PriorityQueue<GridNode> open = new PriorityQueue<>(
                Comparator.comparingDouble(n -> g[n.y][n.x] + h(n.x,n.y,gx,gy))
        );

        g[sy][sx] = 0;
        open.add(new GridNode(sx,sy));

        while(!open.isEmpty()){
            GridNode cur = open.poll();
            if (closed[cur.y][cur.x]) continue;
            closed[cur.y][cur.x] = true;
            if (cur.x==gx && cur.y==gy) return reconstruct(parent, cur);

            for (int[] d : DIRS){
                int nx = cur.x + d[0], ny = cur.y + d[1];
                if(!map.in(nx,ny)) continue;
                double c = map.cost[ny][nx];
                if(!Double.isFinite(c)) continue;

                double step = (d[0]==0 || d[1]==0) ? 1.0 : Math.sqrt(2);
                double ng = g[cur.y][cur.x] + c * step;

                if (ng < g[ny][nx]){
                    g[ny][nx] = ng;
                    parent[ny][nx] = cur;
                    open.add(new GridNode(nx,ny));
                }
            }
        }
        return List.of(); // no path
    }

    private static double h(int x,int y,int gx,int gy){
        double dx = gx - x, dy = gy - y;
        double distIn = Math.hypot(dx,dy) * Field.CELL;
        return distIn / Field.MAX_SPEED_IN_S;
    }

    private static List<int[]> reconstruct(GridNode[][] parent, GridNode end){
        LinkedList<int[]> path = new LinkedList<>();
        for(GridNode n = end; n != null; n = parent[n.y][n.x]) {
            path.addFirst(new int[]{n.x, n.y});
        }
        return path;
    }
}
