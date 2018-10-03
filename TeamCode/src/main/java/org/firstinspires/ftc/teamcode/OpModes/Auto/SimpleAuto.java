package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;


@Autonomous (name = "SimpleAuto", group = "Simple")
public class SimpleAuto extends OpMode {

    DriveTrain dt = new DriveTrain();


    @Override
    public void init() {

        dt.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM);

        dt.resetUpEncoders();

    }

    @Override
    public void start(){

        dt.runMotorEncoders(1440);

        //dt.setThrottle(0.2);

    }

    @Override
    public void loop() {

        dt.checkMotors(0.2);

        telemetry.update();

    }
}

