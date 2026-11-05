package org.firstinspires.ftc.teamcode.Configs.CommandBase.SubSystems;

import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.GATE_OPEN;

import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    private final Servo gate;

    public Gate(Servo gate){
        this.gate = gate;
    }

    public void open(){
        gate.setPosition(GATE_OPEN);
    }

    public void close(){
        gate.setPosition(GATE_CLOSE);
    }

}
