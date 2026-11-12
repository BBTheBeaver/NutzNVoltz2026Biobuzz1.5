package org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;

public class Flywheel {

    private final DcMotorEx motor;

    public Flywheel(DcMotorEx motor) {
        this.motor = motor;
    }

    // Spins up the flywheel
    public void spinUp() {
        motor.setVelocity(1230);
    }

    // Checks whether the flywheel is ready to shoo

    // Stops the flywheel
    public void stop() {
        motor.setVelocity(0);
    }

    // Returns the current flywheel velocity
    public double getVelocity() {
        return motor.getVelocity();
    }
}