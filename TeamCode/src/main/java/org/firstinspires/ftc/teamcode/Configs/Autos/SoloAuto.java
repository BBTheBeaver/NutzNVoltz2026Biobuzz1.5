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

    private final Pose start = poseFactory.of(56.097, 8, 270);
    private final Pose path1 = poseFactory.of(20, 46.747, 180);
    private final Pose point1_5 = poseFactory.of(15, 46.75, 180);
    private final Pose point2Start = poseFactory.of(17, 46.75, 180);
    private final Pose point2 = poseFactory.of(61.2836, 132.4523, 90);
    private final Pose point2Control1 = poseFactory.of(13.7852, 68.8159, 0);
    private final Pose point2Control2 = poseFactory.of(26.6902, 90.2806, 0);
    private final Pose point3 = poseFactory.of(44, 128, 90);
    private final Pose point3Control1 = poseFactory.of(58.51, 115.7, 0);
    private final Pose point4 = poseFactory.of(61.28, 132.45, 90);
    private final Pose point4Control1 = poseFactory.of(58.5, 115.7, 0);
    private final Pose point5Start = poseFactory.of(61.28, 132.45, 90);
    private final Pose point5Top = poseFactory.of(11.7906, 121.9295, 180);
    private final Pose point5Bottom = poseFactory.of(11.7906, 90, 180);
    private final Pose point5Control1 = poseFactory.of(65.1446, 116.9032, 0);

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

                        parallel(
                                instant(intake::on),
                                follow(robot.follower, path1())
                        ),

                        SetRamp.down(ramp),
                        parallel(
                                instant(intake::on),
                                follow(robot.follower, path1_5())
                        ),
                        waitMs(3000),

                        parallel(
                                follow(robot.follower, path2()),
                                sequential(
                                        waitMs(250),
                                        SetRamp.up(ramp)
                                )
                        ),
                        Shoot.create(robot),

                        parallel(
                                instant(intake::on),
                                follow(robot.follower, path3()),
                                sequential(
                                        waitMs(100),
                                        SetRamp.down(ramp)
                                )
                        ),

                        waitMs(3000),

                        SetRamp.up(ramp),
                        follow(robot.follower, path4()),
                        Shoot.create(robot),
                        instant(intake::stop),

                        follow(robot.follower, path5())

                ),
                SpinFlywheel.create(flywheel)
        );
    }

    public Path path1() {
        return Paths.line(m(start), m(path1)).linear(m(start), m(path1));
    }

    public Path path1_5() {
        return Paths.line(path1, point1_5).constant(point1_5);
    }

    public Path path2() {
        return Paths.curve(m(point2Start), m(point2Control1), m(point2Control2), m(point2))
                .linear(m(point2Start), m(point2));
    }

    public Path path3() {
        return Paths.curve(m(point2), m(point3Control1), m(point3)).constant(m(point3));
    }

    public Path path4() {
        return Paths.curve(m(point3), m(point4Control1), m(point4)).constant(m(point4));
    }

    public Path path5() {
        if (AutoConfig.parkTop) {
            return Paths.curve(point5Start, point5Control1, point5Top).linear(point5Start, point5Top);
        }
        return curve(m(point5Start), m(point5Control1), m(point5Bottom)).linear(m(point5Start), m(point5Bottom));
    }
}

