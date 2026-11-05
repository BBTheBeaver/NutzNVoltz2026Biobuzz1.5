package org.firstinspires.ftc.teamcode.Configs.CommandBase.Commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;
import org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;

public class SetIntake {

    public static Command create(Intake intake, Robot robot, Transfer transfer, boolean reverse) {
        return sequential(
                instant(() -> {
                    if (reverse) {
                        intake.reverse();
                        transfer.off();
                        //robot.leftIntakeServo.setPower(1);
                        //robot.rightIntakeServo.setPower(1);
                    } else {
                        intake.on();
                        transfer.on();
                        //.leftIntakeServo.setPower(-1);
                        //.rightIntakeServo.setPower(-1);
                    }
                }),

                waitMs(2000),

                instant(() -> {
                    robot.intake.setPower(0);
                    //robot.leftIntakeServo.setPower(0);
                    //robot.rightIntakeServo.setPower(0);
                })
        );
    }
}