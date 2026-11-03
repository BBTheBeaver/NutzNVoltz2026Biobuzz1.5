package org.firstinspires.ftc.teamcode.Configs.Autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
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

public class SoloAuto implements AutoRoutine {
    private final PoseFactory mirroredFactory = PoseFactory.radians().mirrorX(70.75).mirrorY(70.75);
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
        return m(SOLO_START);
    }

    @Override
    public Command build(Robot robot) {
        Flywheel flywheel = new Flywheel(robot.launcher);
        Intake intake = new Intake(robot.intake);
        Ramp ramp = new Ramp(robot.right_flower, robot.left_flower);

        return deadline(
                sequential(
                        waitMs(2000),
                        Shoot.create(robot),



                        parallel(
                                SetIntake.create(intake, robot, false),
                                follow(robot.follower, FLOWER())
                        ),

                        instant(intake::turnOn),
                        SetRamp.down(ramp),
                        waitMs(500),

                        parallel(
                                SetIntake.create(intake, robot, false),
                                follow(robot.follower, ATFLOWER())

                        ),

                        instant(intake::turnOn),
                        waitMs(2000),

                        parallel(
                                follow(robot.follower, SHOOT()),
                                SetRamp.up(ramp)
                        ),
                        waitMs(500),
                        Shoot.create(robot),

                        instant(intake::stop),
                        follow(robot.follower, PARK())

                ),
                SpinFlywheel.create(flywheel)
        );
    }

    private Path FLOWER() {
        return curve(m(SOLO_START), m(SOLO_FLOWERCURVE), m(SOLO_FLOWER)).linear(m(SOLO_START), m(SOLO_FLOWER));
    }

    private Path ATFLOWER(){
        return line(m(SOLO_FLOWER), m(SOLO_ATFLOWER)).linear(m(SOLO_FLOWER), m(SOLO_ATFLOWER));
    }
    private Path SHOOT() {
        return curve(m(SOLO_FLOWER), m(SOLO_SHOOTCURVE1), m(SOLO_SHOOTCURVE2), m(SOLO_SHOOTCURVE3), m(SOLO_SHOOT)).linear(m(SOLO_ATFLOWER), m(SOLO_SHOOT));
    }

    private Path PARK() {
        return curve(m(SOLO_SHOOT), m(SOLO_PARKCURVE), m(SOLO_PARK)).linear(m(SOLO_SHOOT), m(SOLO_PARK));
    }

    private Path PARK(){
        if (AutoConfig.parkTop) {
            return curve(m(SOLO_SHOOT), m(SOLO_PARKCURVE), m(SOLO_PARK)).linear(m(SOLO_SHOOT), m(SOLO_PARK));
        }
        return curve(m(SOLO_SHOOT), m(SOLO_PARKCURVE), m(SOLO_PARKBOTTOM)).linear(m(SOLO_SHOOT), m(SOLO_PARKBOTTOM));
    }
}

