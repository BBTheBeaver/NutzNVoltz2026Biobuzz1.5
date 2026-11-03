package org.firstinspires.ftc.teamcode.Configs.Autos;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.paths.interpolator.Interpolator.linear;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Poses.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.SetIntake;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.SetRamp;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.SpinFlywheel;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Ramp;
import org.firstinspires.ftc.teamcode.Configs.Globals.AutoConfig;
import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;


public class ParkAuto implements AutoRoutine {
    private final PoseFactory mirroredFactory = PoseFactory.radians().mirrorX(70.75).mirrorY(70.75);
    private final boolean blue;

    public ParkAuto(boolean blue) {
        this.blue = blue;
    }

    private Pose m(Pose pose) {
        if (blue) {
            return mirroredFactory.of(pose.x(), pose.y(), pose.heading());
        }
        return pose;
    }

    @Override
    public Pose getStartPose() {
        return m(PARK_START);
    }


    @Override
    public Command build(Robot robot) {
        Flywheel flywheel = new Flywheel(robot.launcher);
        Intake intake = new Intake(robot.intake);
        Ramp ramp = new Ramp(robot.right_flower, robot.left_flower);
        return sequential(
                follow(robot.follower, PARK())
        );
    }
    private Path PARK(){
        if (AutoConfig.parkTop) {
            return line(m(PARK_START), m(PARK_PARK)).constant(m(PARK_START));
        }
        return line(m(PARK_START), m(PARK_PARKBOTTOM)).constant(m(PARK_PARKBOTTOM));
    }
}

