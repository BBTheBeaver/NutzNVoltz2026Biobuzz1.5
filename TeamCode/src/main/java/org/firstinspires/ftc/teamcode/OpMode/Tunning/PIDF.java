package org.firstinspires.ftc.teamcode.OpMode.Tunning;

import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.GATE_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

@TeleOp(name = "PIDF Tuner", group = "Tuning")
public class PIDF extends OpMode {

    private Robot robot;
    private DcMotorEx launcher;   // change if your launcher field has a different type/name

    // Start with everything off except a rough F
    private double p = 0, i = 0, d = 0, f = 11.30;
    private double targetVelocity = 1230; // ticks/sec
    private boolean running = false;

    private final String[] names = {"F", "P", "I", "D"};
    private int selected = 0; // start on F

    private double step = 1.0; // multiplier for the value step

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        launcher = robot.launcher;
        applyPIDF();
    }

    private void applyPIDF() {
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, f)
        );
    }

    private double baseStep(int index) {
        // F, P, I, D
        switch (index) {
            case 0: return 0.1;   // F
            case 1: return 1.0;   // P
            case 2: return 0.1;   // I
            default: return 0.1;  // D
        }
    }

    private void change(double amount) {
        switch (selected) {
            case 0: f = Math.max(0, f + amount); break;
            case 1: p = Math.max(0, p + amount); break;
            case 2: i = Math.max(0, i + amount); break;
            case 3: d = Math.max(0, d + amount); break;
        }
        applyPIDF();
    }

    @Override
    public void loop() {

        if(gamepad1.y){
            robot.transfer.setPower(1);
            robot.intake.setPower(1);
            robot.gate.setPosition(GATE_OPEN);
        }
        else{
            robot.transfer.setPower(0);
            robot.intake.setPower(0);
            robot.gate.setPosition(GATE_CLOSE);
        }

        // Select which value to edit
        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % names.length;
        if (gamepad1.dpadLeftWasPressed())  selected = (selected - 1 + names.length) % names.length;

        // Change selected value
        if (gamepad1.dpadUpWasPressed())   change(baseStep(selected) * step);
        if (gamepad1.dpadDownWasPressed()) change(-baseStep(selected) * step);

        // Step size (x0.1 / x10)
        if (gamepad1.leftBumperWasPressed())  step = Math.max(0.01, step / 10.0);
        if (gamepad1.rightBumperWasPressed()) step = Math.min(100, step * 10.0);

        // Target velocity
        if (gamepad1.bWasPressed()) targetVelocity += 100;
        if (gamepad1.xWasPressed()) targetVelocity = Math.max(0, targetVelocity - 100);

        // Start / stop
        if (gamepad1.aWasPressed()) running = !running;

        if (running) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setPower(0);
        }

        double measured = launcher.getVelocity();
        double error = targetVelocity - measured;

        telemetry.addLine("DPAD L/R pick value, UP/DOWN change it");
        telemetry.addLine("Bumpers change step size, X/Y change target, A start/stop");
        telemetry.addLine();
        for (int n = 0; n < names.length; n++) {
            double v = n == 0 ? f : n == 1 ? p : n == 2 ? i : d;
            telemetry.addData((n == selected ? "> " : "  ") + names[n], "%.3f", v);
        }
        telemetry.addData("Step multiplier", step);
        telemetry.addLine();
        telemetry.addData("Running", running);
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Measured", measured);
        telemetry.addData("Error", error);
        if (measured > 1 && running) {
            telemetry.addData("Suggested F (if P/I/D = 0)", "%.2f", f * targetVelocity / measured);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (launcher != null) launcher.setPower(0);
    }
}