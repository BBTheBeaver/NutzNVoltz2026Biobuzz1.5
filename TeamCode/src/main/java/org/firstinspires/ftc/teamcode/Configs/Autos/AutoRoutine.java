package org.firstinspires.ftc.teamcode.Configs.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

public interface AutoRoutine {

    Pose getStartPose();
    Command build(Robot robot);


}