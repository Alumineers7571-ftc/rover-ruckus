package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "MoveTOGround", group = "test")
public class MoveToGround extends LinearOpMode {

    Robot rb = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.MECANUM, true);

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData(rb.hanger.getDistanceToGround(DistanceUnit.INCH) + " in", " ");
            telemetry.addData("Touched: ", rb.hanger.isTouched());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !rb.hanger.moveToLowerLimit()){
            telemetry.addData(rb.hanger.getDistanceToGround(DistanceUnit.INCH) + " in", " ");
            telemetry.update();
        }

        rb.hanger.controlHanger(0);

    }
}
