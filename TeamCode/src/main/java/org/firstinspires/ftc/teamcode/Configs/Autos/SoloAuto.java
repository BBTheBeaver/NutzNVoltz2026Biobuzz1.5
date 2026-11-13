package org.firstinspires.ftc.teamcode.Configs.Autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.SetRamp;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands.SpinFlywheel;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Ramp;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.Configs.Globals.AutoConfig;
import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

public class SoloAuto implements AutoRoutine{
    private final PoseFactory mirroredFactory = PoseFactory.radians().mirrorX(70.75).mirrorY(70.75);

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.9101, 9.1679, 270);
    private final Pose path1 = poseFactory.of(30, 46.747, 180);
    private final Pose point2 = poseFactory.of(12.5, 46.75, 180);
    private final Pose point3Start = poseFactory.of(13, 46.75, 180);
    private final Pose point3 = poseFactory.of(61.2836, 132.4523, 90);
    private final Pose point3Control1 = poseFactory.of(43.2133, 44.783, 0);
    private final Pose point3Control2 = poseFactory.of(2.2071, 55.7027, 0);
    private final Pose point3Control3 = poseFactory.of(10.1468, 114.2296, 0);
    private final Pose point3Control4 = poseFactory.of(61.4295, 111.8494, 0);
    private final Pose point4 = poseFactory.of(46.5, 115, 90);
    private final Pose point4Control1 = poseFactory.of(58.51, 115.7, 0);
    private final Pose point5 = poseFactory.of(46.5, 128.1, 90);
    private final Pose point6 = poseFactory.of(61.28, 132, 90);
    private final Pose point6Control1 = poseFactory.of(52.2465, 100.2503, 0);
    private final Pose point7Start = poseFactory.of(61.28, 132, 90);
    private final Pose point7Top = poseFactory.of(11.7906, 121.9295, 180);
    private final Pose point7Bottom = poseFactory.of(11.7906, 90, 180);
    private final Pose point7Control1 = poseFactory.of(65.1446, 116.9032, 0);

    private final boolean blue;
    public SoloAuto(boolean blue) {
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
                        waitMs(1000),
                        Shoot.create(robot),

                        follow(robot.follower, path1()),

                        SetRamp.down(ramp),
                        waitMs(1000),
                        parallel(
                                instant(intake::on),
                                follow(robot.follower, path2())
                        ),
                        waitMs(3000),

                        parallel(
                                follow(robot.follower, path3()),
                                sequential(
                                        waitMs(100),
                                        SetRamp.up(ramp),
                                        waitMs(500),
                                        instant(intake::stop)
                                )
                        ),


                        Shoot.create(robot),

                        parallel(
                                instant(intake::on),
                                follow(robot.follower, path4())

                        ),

                        SetRamp.down(ramp),
                        waitMs(1000),
                        parallel(
                                follow(robot.follower, path5())
                        ),
                       waitMs(3000),

                       parallel(
                               sequential(
                                       waitMs(100),
                                       SetRamp.up(ramp)
                               ),
                            follow(robot.follower, path6())
                       ),

                        Shoot.create(robot),
                        instant(intake::stop),

                        follow(robot.follower, path7())

                ),
                SpinFlywheel.create(flywheel)
        );
    }

    public Path path1() {
        return Paths.line(m(start), m(path1)).linear(m(start), m(path1));
    }

    public Path path2() {
        return Paths.line(m(path1), m(point2)).constant(m(point2));
    }

    public Path path3() {
        return Paths.curve(m(point3Start), m(point3Control1), m(point3Control2), m(point3Control3), m(point3Control4), m(point3)).linear(m(point3Start), m(point3));
    }

    public Path path4() {
        return Paths.curve(m(point3), m(point4Control1), m(point4)).constant(m(point4));
    }

    public Path path5() {
        return Paths.line(m(point4), m(point5)).constant(m(point5));
    }

    public Path path6() {
        return Paths.curve(m(point5), m(point6Control1), m(point6)).constant(m(point6));
    }

    public Path path7() {
        if (AutoConfig.parkTop) {
            return Paths.curve(m(point7Start), m(point7Control1), m(point7Top)).linear(m(point7Start), m(point7Top));
        }
        return Paths.curve(m(point7Start), m(point6Control1), m(point7Bottom)).linear(m(point7Start), m(point7Bottom));
    }
}

