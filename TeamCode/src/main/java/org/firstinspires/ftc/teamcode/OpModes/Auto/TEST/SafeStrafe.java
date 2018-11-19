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

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM, true);

        while(!isStarted()){

            telemetry.addData("angle", rb.drive.getGyroangle());
            telemetry.update();

        }

        float startingGyroAngle = (float)rb.drive.getGyroangle();

        waitForStart();

        rb.drive.safeStrafe(startingGyroAngle, true, telemetry, 0.3);
        sleep(3000);
        rb.drive.setThrottle(0);
    }

}
