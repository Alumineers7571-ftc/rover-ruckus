package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.CV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class AutoAlign extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM);

        waitForStart();

        while(opModeIsActive() && !robot.computerVision.isAligned()){
            robot.drive.strafe(0.3);
        }

    }
}
