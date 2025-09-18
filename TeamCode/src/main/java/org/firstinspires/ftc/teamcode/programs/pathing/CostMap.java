package org.firstinspires.ftc.teamcode.programs.pathing;

public class CostMap {
    // cost[y][x] — 1 = free, + = costly, INF = blocked
    public final double[][] cost = new double[Field.NY][Field.NX];

    public CostMap() {
        for (int y=0;y<Field.NY;y++) {
            for (int x=0;x<Field.NX;x++) cost[y][x] = 1.0;
        }
    }

    public boolean in(int x,int y){ return x>=0 && x<Field.NX && y>=0 && y<Field.NY; }

    public static int toGX(double xcm){ return (int)Math.round((xcm + Field.HALF) / Field.CELL); }
    public static int toGY(double ycm){ return (int)Math.round((ycm + Field.HALF) / Field.CELL); }
    public static double toXcm(int gx){ return gx*Field.CELL - Field.HALF; }
    public static double toYcm(int gy){ return gy*Field.CELL - Field.HALF; }

    /** Block a rectangle given center+size in cm */
    public void blockRectCm(double cx,double cy,double w,double h){
        int x0 = toGX(cx - w/2), x1 = toGX(cx + w/2);
        int y0 = toGY(cy - h/2), y1 = toGY(cy + h/2);
        for(int y=y0;y<=y1;y++){
            for(int x=x0;x<=x1;x++){
                if(in(x,y)) cost[y][x] = Double.POSITIVE_INFINITY;
            }
        }
    }

    /** Inflate all blocked cells by an integer radius (in cells) as a safety buffer */
    public void inflate(int radiusCells){
        boolean[][] blocked = new boolean[Field.NY][Field.NX];
        for(int y=0;y<Field.NY;y++) for(int x=0;x<Field.NX;x++)
            blocked[y][x] = !Double.isFinite(cost[y][x]);

        for(int y=0;y<Field.NY;y++){
            for(int x=0;x<Field.NX;x++){
                if (blocked[y][x]){
                    for(int dy=-radiusCells; dy<=radiusCells; dy++){
                        for(int dx=-radiusCells; dx<=radiusCells; dx++){
                            int nx = x+dx, ny=y+dy;
                            if(in(nx,ny)) cost[ny][nx] = Double.POSITIVE_INFINITY;
                        }
                    }
                }
            }
        }
    }
}
