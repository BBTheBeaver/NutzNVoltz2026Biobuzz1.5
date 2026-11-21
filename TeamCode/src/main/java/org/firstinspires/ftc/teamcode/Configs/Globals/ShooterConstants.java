package org.firstinspires.ftc.teamcode.Configs.Globals;

public class ShooterConstants {

    private static final double MIN_SPEED = 0;  // placeholders, use your tested values
    private static final double MAX_SPEED = 1500;

    // Calculates flywheel speed using a polynomial
    public static double flywheelSpeed(double x) {
        double speed = (((-0.00378682 * x + 0.442826) * x - 17.72511) * x + 274.43342) * x;
        return clamp(speed, MIN_SPEED, MAX_SPEED);
    }


    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}