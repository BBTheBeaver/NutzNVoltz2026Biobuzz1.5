package org.firstinspires.ftc.teamcode.Configs.Autos;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.paths.interpolator.Interpolator.linear;

import com.pedropathing.api.Paths;
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
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.Configs.Globals.AutoConfig;
import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

public class FarAuto implements AutoRoutine{

    private final PoseFactory mirroredFactory = PoseFactory.radians().mirrorX(70.75).mirrorY(70.75);

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(61.1499, 132.4562, 90);
    private final Pose path1 = poseFactory.of(47.049, 128.9437, 90);
    private final Pose path1Control1 = poseFactory.of(54.0127, 120.6964, 0);
    private final Pose point2 = poseFactory.of(61, 132.5, 90);
    private final Pose point2Control1 = poseFactory.of(54.01, 120.7, 0);
    private final Pose point3Top = poseFactory.of(12.425, 122.4423, 180);
    private final Pose point3Bottom = poseFactory.of(12.425, 90, 180);
    private final Pose point3Control1 = poseFactory.of(70.8566, 119.4897, 0);

    private final boolean blue;
    public FarAuto(boolean blue) {
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
        return m(start);
    }

    @Override
    public Command build(Robot robot) {
        Flywheel flywheel = new Flywheel(robot.launcher);
        Intake intake = new Intake(robot.intake);
        Ramp ramp = new Ramp(robot.left_flower, robot.right_flower);
        Transfer transfer = new Transfer(robot.transfer);
        return deadline(
             sequential(
                     waitMs(8000),
                     Shoot.create(robot),

                     parallel(
                             instant(intake::on),
                             follow(robot.follower, path1()),
                             sequential(
                                     waitMs(500),
                                     SetRamp.down(ramp)
                             )
                     ),

                     SetRamp.up(ramp),
                     follow(robot.follower, path2()),
                     Shoot.create(robot),
                     instant(intake::stop),

                     follow(robot.follower, path3())

            ),
            SpinFlywheel.create(flywheel)
        );
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).constant(point2);
    }

    public Path path3() {
        if (AutoConfig.parkTop) {
            return Paths.curve(point2, point3Control1, point3Top).linear(point2, point3Top);
        }
        return curve(m(point2), m(point3Control1), m(point3Bottom)).linear(m(point2), m(point3Bottom));
    }

}
