package org.firstinspires.ftc.teamcode.OpMode.Tunning;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.controllers.PIDController;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

import java.util.Objects;


@TeleOp(name = "ShooterTuner", group = "StarterBot")
public class ShooterTunner extends OpMode {
    int Shooter_speed = 1230;
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        robot.periodic();
        Scheduler.execute(); // runs anything scheduled — nothing's scheduled yet, but it's plugged in and ready

        robot.launcher.setVelocity(Shooter_speed);

        robot.intake.setPower(1);

        if(gamepad1.y){
            robot.transfer.setPower(1);
            robot.gate.setPosition(GATE_OPEN);
        }
        else{
            robot.transfer.setPower(0);
            robot.gate.setPosition(GATE_CLOSE);
        }


        if (gamepad1.dpadUpWasPressed()) {
            Shooter_speed += 100;
        }
        if (gamepad1.dpadDownWasPressed()) {
            Shooter_speed -= 100;
        }
        if (gamepad1.xWasPressed()) {
            Shooter_speed -= 10;
        }
        if (gamepad1.bWasPressed()) {
            Shooter_speed += 10;
        }

        telemetry.addData("Speed", Shooter_speed);
        telemetry.addLine("Dpad up/down for increments of 100");
        telemetry.addLine("Dpad x/b for increments of 10");
    }
}
