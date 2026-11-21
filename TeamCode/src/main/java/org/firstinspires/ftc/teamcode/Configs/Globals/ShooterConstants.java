
package org.firstinspires.ftc.teamcode.Configs.Globals;

public class ShooterConstants {

    // ═════════════════════════ CONSTANTS ═════════════════════════

    // Distances in inches, sorted smallest to largest
    private static final double[] DIST = { 24, 48, 72, 96, 120 };

    // Launcher velocity (ticks/sec) for each distance above
    private static final double[] SPEED = { 1100, 1150, 1200, 1250, 1300 };


    // ═════════════════════════ METHODS ═════════════════════════

    // Returns launcher speed based on distance using interpolation
    public static double speedFor(double d) {
        if (d <= DIST[0]) return SPEED[0];
        if (d >= DIST[DIST.length - 1]) return SPEED[SPEED.length - 1];

        //region Find speed between distances
        for (int i = 0; i < DIST.length - 1; i++) {
            if (d <= DIST[i + 1]) {
                double t = (d - DIST[i]) / (DIST[i + 1] - DIST[i]);
                return SPEED[i] + t * (SPEED[i + 1] - SPEED[i]);
            }
        }
        //endregion

        return SPEED[SPEED.length - 1];
    }

    // Calculates flywheel speed using a polynomial
    public static double flywheelSpeed(double x) {
        return (((-0.00021331 * x + 0.0395582) * x - 2.57344) * x + 76.78332) * x + 347.25784;
    }
}
