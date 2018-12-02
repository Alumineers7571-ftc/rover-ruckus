package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hanger{

    DcMotorEx hanger;

    DistanceSensor distanceSensor;

    final double SENSOR_IN_FROM_GROUND = 1;

    public Hanger(){
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        hanger = hardwareMap.get(DcMotorEx.class,"hanger");

        hanger.setDirection(DcMotorSimple.Direction.REVERSE); //its geared

        distanceSensor = hardwareMap.get(DistanceSensor.class, "range");

        double startingDistanceIn = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean scored = false;

        if(startingDistanceIn < 4){
            scored = false;
        } else {
            scored = true;
        }

        telemetry.addData("4+ ?: ", scored);

    }

    public void controlHanger(Gamepad gamepad){

        hanger.setPower(gamepad.left_stick_y);

    }

    public void controlHanger(double power){

        hanger.setPower(power);

    }

    public boolean moveToGround(){

        if((distanceSensor.getDistance(DistanceUnit.INCH) > (0 + SENSOR_IN_FROM_GROUND))){

            hanger.setPower(0);
            return true;

        } else {

            hanger.setPower(0.3);
            return false;

        }


    }


}
