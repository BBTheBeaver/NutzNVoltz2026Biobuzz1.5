package org.firstinspires.ftc.teamcode.Configs.Globals;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Poses {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    //Pose reset
    public static final Pose BLUE_RESET = poseFactory.of(130, 132, 270);
    public static final Pose RED_RESET = poseFactory.of(10, 10, 90);


    //Small solo sequence start
    public static final Pose SOLO_START = poseFactory.of(56.6, 10, 270);

    //Small solo sequence
    public static final Pose SOLO_FLOWER = poseFactory.of(22.4071, 47.1358, 180);
    public static final Pose SOLO_FLOWERCURVE = poseFactory.of(29.9019, 47.9481, 0);
    public static final Pose SOLO_ATFLOWER = poseFactory.of(13, 47.1472, 180);
    public static final Pose SOLO_SHOOT = poseFactory.of(61.4057, 131.2094, 90);
    public static final Pose SOLO_SHOOTCURVE1 = poseFactory.of(35.4584, 24.7886, 0);
    public static final Pose SOLO_SHOOTCURVE2 = poseFactory.of(1.5351, 107.8854, 0);
    public static final Pose SOLO_SHOOTCURVE3 = poseFactory.of(17.8311, 74.9788, 0);
    public static final Pose SOLO_PARK = poseFactory.of(14.2175, 92.4, 0);
    public static final Pose SOLO_PARKCURVE = poseFactory.of(15.1707, 78.0726, 0);
    public static final Pose SOLO_PARKBOTTOM = poseFactory.of(14, 93.24, 270);




    // Far sequence start
    public static final Pose FAR_START = poseFactory.of(61.5, 132.5812, 90);

    // Far sequence
    public static final Pose FAR_FLOWER = poseFactory.of(47.2575, 120.5156, 90);
    public static final Pose FAR_FLOWERCURVE = poseFactory.of(58.9226, 121.4925, 0);
    public static final Pose FAR_ATFLOWER = poseFactory.of(47, 128, 90);
    public static final Pose FAR_SHOOT = poseFactory.of(61.5, 132.5812, 90);
    public static final Pose FAR_SHOOTCURVE = poseFactory.of(54.5175, 109.0778, 0);
    public static final Pose FAR_PARKTOP = poseFactory.of(14.0627, 120.5764, 360);
    public static final Pose FAR_PARKCURVE = poseFactory.of(37.583, 97.5858, 0);
    public static final Pose FAR_PARKBOTTOM = poseFactory.of(10.63, 92.25, 0);

    //Park sequence start
    public static final Pose PARK_START = poseFactory.of(10, 132, 270);

    //Park sequence
    public static final Pose PARK_PARK = poseFactory.of(14.2175, 121.167, 270);
    public static final Pose PARK_PARKBOTTOM = poseFactory.of(14, 93.24, 270);


}