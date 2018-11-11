package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamMarkerer extends BaseHardware{

    Servo teamMarkerer;

    final double POS_DOWN = 1;
    final double POS_UP = 0.4;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        teamMarkerer = hardwareMap.servo.get("tm");

        telemetry.addLine("tm good");
        telemetry.update();

    }

    public void setTMDown(){
        teamMarkerer.setPosition(POS_DOWN);
    }

    public void setTMUp(){
        teamMarkerer.setPosition(POS_UP);
    }

    public Servo getTeamMarkerer(){
        return teamMarkerer;
    }

}
