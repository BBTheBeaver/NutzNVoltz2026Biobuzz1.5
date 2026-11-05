package org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    private final DcMotor transfer;

    public Transfer(DcMotor transfer){
        this.transfer = transfer;
    }

    public void on(){
        transfer.setPower(1);
    }

    public void off(){
        transfer.setPower(0);
    }
}
