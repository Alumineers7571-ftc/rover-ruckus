package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "MoveTOGround", group = "test")
public class MoveToGround extends LinearOpMode {

    Robot rb = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM, true);

        waitForStart();

        while (opModeIsActive()){
            rb.hanger.moveToGround();
        }

        rb.hanger.controlHanger(0);

    }
}
