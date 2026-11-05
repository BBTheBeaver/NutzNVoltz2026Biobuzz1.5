package org.firstinspires.ftc.teamcode.Configs.Globals;


import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.FLOWER_RAMP_UP;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Robot {
    public final Follower follower;
    public final List<LynxModule> hubs;
    public final Timer loop = new Timer();
    public double loops = 0, lastloop = 0, loopTime = 0;

    //hardware
    public final DcMotorEx launcher;
    public final DcMotor intake;
    //public final CRServo leftIntakeServo;
    //public final CRServo rightIntakeServo;
    public final DcMotor transfer;

    public final Servo right_flower;
    public final Servo left_flower;

    public final Servo gate;




    public Robot(HardwareMap hw){
        follower = PedroConstants.createFollower(hw);
        hubs = hw.getAll(LynxModule.class);
        for(LynxModule hub:hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        hw.get(DcMotor.class, "lf").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "rf").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "lr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "rr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // ----- Non-drivetrain hardware init -----
        intake = hw.get(DcMotor.class, "intake");
        launcher = hw.get(DcMotorEx.class, "launcher");
        transfer = hw.get(DcMotor.class, "transfer");
        //leftIntakeServo = hw.get(CRServo.class, "left_intake_servo");
        //rightIntakeServo = hw.get(CRServo.class, "right_intake_servo");

        right_flower = hw.get(Servo.class, "right_flower");
        left_flower = hw.get(Servo.class, "left_flower");
        gate = hw.get(Servo.class, "gate");


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        //leftIntakeServo.setPower(0);
        //rightIntakeServo.setPower(0);

        left_flower.setDirection(Servo.Direction.REVERSE);



        right_flower.setPosition(FLOWER_RAMP_UP);
        left_flower.setPosition(FLOWER_RAMP_UP);
        gate.setPosition(GATE_CLOSE);

        //rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        // -----------------------------------------

        loop.reset();
        periodic();
    }

    public void periodic(){
        loops++;
        if(loops > 10){
            double now = loop.get(TimeUnit.MICROSECONDS);
            loopTime = (now-lastloop) / loops;
            lastloop = now;
            loops = 0;
        }
        follower.update();
    }
    public double getLoopTimeMs() { return loopTime;}
    public double getLoopTimeHz() { return 1000/loopTime;}

}