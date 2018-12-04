package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "Encoder Drive")
public class EncoderDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();

        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK, true);

        robot.drive.kill();

        waitForStart();

        robot.drive.encoderDrive(0.5, 19 ,19, opModeIsActive());

        telemetry.addData("counts", robot.drive.FL.getCurrentPosition() + " " + robot.drive.FR.getCurrentPosition() + " " + robot.drive.BL.getCurrentPosition() + " " + robot.drive.BR.getCurrentPosition());
        telemetry.update();



    }

}
