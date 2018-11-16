package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "SafeStrafe", group = "TEST")
public class SafeStrafe extends LinearOpMode{

    Robot rb = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM);

        waitForStart();

        rb.drive.adjustHeading(90, true);

        while(opModeIsActive() && !isStopRequested()) {

            rb.drive.safeStrafe(90, true, telemetry, 0.5);

            idle();

        }
    }

}
