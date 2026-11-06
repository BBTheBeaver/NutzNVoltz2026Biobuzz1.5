package org.firstinspires.ftc.teamcode.Configs.Globals;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Constants {
    public static final int LAUNCHER_TARGET_VELOCITY = 1140; //2678 RPM 1150
    public static final int LAUNCHER_MIN_VELOCITY = 1110; //2571 RPM 1100

    public static final double INTAKE_FORWARD = 1.0;
    public static final double INTAKE_REVERSE = -0.5;

    // ─── Goal pose [x_inches, y_inches] (Pedro field coords) ─────────────────
    // Blue alliance default — mirrored for Red in TeleOp initialize()
    public static double[] GOAL_POSE_BLUE = {84.0,  47.0};//-50 For the other side
    public static double[] GOAL_POSE_RED  = {58.0, 47.0};//-50 for the other side

    public static final double FLOWER_RAMP_UP = 0;
    public static final double FLOWER_RAMP_DOWN = 1;

    public static final double GATE_OPEN = 1;
    public static final double GATE_CLOSE = 0;

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    //Pose reset
    public static final Pose RESET_GARDEN = poseFactory.of(10, 10, 90);
    public static final Pose RESET_FLOWER = poseFactory.of(8.9151, 34.6079, 0);
    public static final Pose RESET_TOP = poseFactory.of(9.7734, 132.3336, 270);

    public static double OTHERSIDE;

}
