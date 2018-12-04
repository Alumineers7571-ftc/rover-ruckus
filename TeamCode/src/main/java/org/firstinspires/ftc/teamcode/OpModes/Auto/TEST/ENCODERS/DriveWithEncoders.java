package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "Drive With Encoders")
public class DriveWithEncoders extends LinearOpMode {

    DcMotorEx FL, FR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Robot robot = new Robot();

        //robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK, true);

        //robot.drive.kill();

        waitForStart();

        BL.setTargetPosition(-1440);

        setThrottle(0.25);

        while(opModeIsActive() && BL.getCurrentPosition() < BL.getTargetPosition()){
            telemetry.addData("count: ", BL.getCurrentPosition());
            idle();
        }

        setThrottle(0);


    }


    public void setThrottle(double power){
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }


}
