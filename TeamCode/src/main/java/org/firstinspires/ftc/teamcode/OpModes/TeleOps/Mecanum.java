package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@TeleOp (name = "Mecanum", group = "MISC")
public class Mecanum extends OpMode{

    Robot robot = new Robot();

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM);

    }

    @Override
    public void loop() {

        robot.drive.manualDrive(gamepad1);
        //robot.glyphSystem.controlSystem(gamepad2);
        robot.mineralSystem.controlSystem(gamepad2, telemetry);

    }

}
