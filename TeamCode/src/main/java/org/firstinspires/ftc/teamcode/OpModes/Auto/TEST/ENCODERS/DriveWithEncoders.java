package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "Drive With Encoders")
public class DriveWithEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();

        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);

        robot.drive.kill();

        waitForStart();

        robot.drive.moveEncoder(12, 12, 0.4);

        while(opModeIsActive() && !isStopRequested()) {

            if(!robot.drive.motorsBusy()){

                robot.drive.setThrottle(0);

            }


        }



    }

}
