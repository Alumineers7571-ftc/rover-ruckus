package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class DepotShell extends LinearOpMode{

    Robot rb = new Robot();

    BaseAuto autoExec;

    @Override
    public void runOpMode() throws InterruptedException {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM, true);

        autoExec = new BaseAuto(rb, DepotShell.this);

        autoExec.SetUp();

        waitForStart();

        autoExec.DropDown();
        autoExec.MoveToSample();

    }
}
