package org.firstinspires.ftc.teamcode.Configs.Globals;

public class ShooterTable {

    // distances in inches, must be sorted smallest to largest
    private static final double[] DIST  = { 24,   48,   36.2,   96,   120  };

    // launcher velocity (ticks/sec) that works at each distance above
    private static final double[] SPEED = {  1100, 1150, 1230, 1250, 1300 };

    public static double speedFor(double d) {
        if (d <= DIST[0]) return SPEED[0];
        if (d >= DIST[DIST.length - 1]) return SPEED[SPEED.length - 1];

        for (int i = 0; i < DIST.length - 1; i++) {
            if (d <= DIST[i + 1]) {
                double t = (d - DIST[i]) / (DIST[i + 1] - DIST[i]);
                return SPEED[i] + t * (SPEED[i + 1] - SPEED[i]);
            }
        }
        return SPEED[SPEED.length - 1];
    }
}