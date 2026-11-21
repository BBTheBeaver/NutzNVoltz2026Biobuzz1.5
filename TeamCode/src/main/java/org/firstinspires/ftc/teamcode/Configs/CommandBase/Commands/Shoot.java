package org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Flywheel;

public class Shoot {

    public static Command create(Robot robot) {
        return sequential(
                instant(() -> {
                    robot.transfer.setPower(0.75);
                    robot.gate.setPosition(1);
                    robot.intake.setPower(1);
                }),

                waitMs(2400),

                instant(() -> {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    robot.gate.setPosition(0);
                })
        );
    }
}