package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ThreadPool;

@Autonomous
public class SimpleEncoderTest extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "test");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        while(opModeIsActive()){

            telemetry.addLine("count: " + motor.getCurrentPosition());
            motor.setPower(0.4);
            telemetry.update();

        }




    }
}
