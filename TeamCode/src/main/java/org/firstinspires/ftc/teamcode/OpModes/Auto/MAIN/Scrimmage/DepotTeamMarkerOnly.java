package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN.Scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "DepotTMOnly", group = "MAIN")
@Disabled
public class DepotTeamMarkerOnly extends LinearOpMode {

    Robot rb = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {


        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM);

        rb.tm.setTMUp();

        telemetry.addData("ALL SYSTEMS", "GO");
        telemetry.update();

        waitForStart();

        rb.drive.setThrottle(0.6);
        sleep(2500);
        rb.drive.setThrottle(0);

        rb.tm.setTMDown();

        sleep(200);
        rb.drive.setThrottle(-1);
        sleep(500);
        rb.drive.setThrottle(0);
        sleep(500);
        stop();

    }


}
