
package org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems;

import com.pedropathing.controllers.PIDController;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

public class Aim {

    // ═════════════════════════ CONSTANTS ═════════════════════════

    private final PIDController headingController = new PIDController(2.5, 0.0, 0.1);


    // ═════════════════════════ VARIABLES ═════════════════════════

    private double headingError = 0;
    private double distanceToGoal = 0;


    // ═════════════════════════ AIM UPDATE ═════════════════════════

    public double update(Robot robot, boolean headingLock, double[] goalPose, double stickTurn) {
        double x = robot.follower.pose().x();
        double y = robot.follower.pose().y();
        double tof = 0.5;

        double goalY = y > 75 ? 97.0 : 47.0;
        double vgx = goalPose[0] - robot.follower.velocity().vx * tof;
        double vgy = goalY - robot.follower.velocity().vy * tof;

        double dx = vgx - x;     // robot to virtual goal
        double dy = vgy - y;
        distanceToGoal = Math.hypot(dx, dy);

        if (!headingLock) {
            headingError = 0;
            return stickTurn;
        }

        double kV = -0.75; // tune sign/magnitude while strafing
        double denom = dx * dx + dy * dy;
        double headingVelFF = denom > 1e-6
                ? (dx * robot.follower.velocity().vy - dy * robot.follower.velocity().vx) / denom
                : 0;

        headingError = normalizeAngle(Math.atan2(dy, dx) + Math.PI - robot.follower.pose().heading());

        if (Math.abs(headingError) < Math.toRadians(1)) {
            return 0;
        }

        return headingController.calculate(0, headingError) + kV * headingVelFF;
    }


    // ═════════════════════════ GETTERS ═════════════════════════

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getHeadingError() {
        return headingError;
    }


    // ═════════════════════════ HELPER METHODS ═════════════════════════

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
