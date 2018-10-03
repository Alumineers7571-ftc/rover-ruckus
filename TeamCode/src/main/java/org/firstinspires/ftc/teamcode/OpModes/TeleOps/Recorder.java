package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.FinalValues;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.io.File;

@TeleOp(name = "Recorder", group = "input")
public class Recorder extends OpMode{

    boolean isStarted = false;

    File file;
    Robot robot = new Robot();

    @Override
    public void init() {

        robot.init(hardwareMap,telemetry, DriveTrain.DriveTypes.TANK);

    }

    @Override
    public void init_loop(){


    }


    @Override
    public void loop() {

        if(gamepad1.y){
            file = FinalValues.cbStartToTM;
            robot.inputManager.setupRecording(file);
            isStarted = true;
        } else if (gamepad1.x){
            file = FinalValues.cbTMtoCrater;
            robot.inputManager.setupRecording(file);
            isStarted = true;
        }

        if (isStarted){
            robot.inputManager.writeInputs(gamepad1);
        }

    }
}
