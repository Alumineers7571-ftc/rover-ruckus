package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MineralSystem{

    DcMotor extendo;

    CRServo intakeLeft, intakeRight;

    private static final double SERVO_LITTLE_POS = .25;
    private static final double SERVO_UP_POS = 1;
    private static final double SERVO_DOWN_POS = 0;

    private boolean servoSet = false;

    private boolean isIntakeRunning = false;
    private boolean isHangerRunning = false;
    private boolean isExtendoRunning = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        extendo = hardwareMap.dcMotor.get("extendo");

        intakeLeft = hardwareMap.crservo.get("intakeL");
        intakeRight = hardwareMap.crservo.get("intakeR");

        telemetry.addLine(getClass().toString() + " setup");
        telemetry.update();
    }

    public void controlSystem(Gamepad gamepad, Telemetry telemetry){


        extendo.setPower(gamepad.right_stick_y);

        intakeLeft.setPower(-(gamepad.left_trigger - gamepad.right_trigger));
        intakeRight.setPower((gamepad.left_trigger - gamepad.right_trigger));


    }

    public void runIntake(double power){

        if (power != 0){
            isIntakeRunning = true;
        } else {
            isIntakeRunning = false;
        }

        //positive power is intaking
        //negative power is outtaking

        intakeLeft.setPower(-power);
        intakeRight.setPower(power);



    }
}
