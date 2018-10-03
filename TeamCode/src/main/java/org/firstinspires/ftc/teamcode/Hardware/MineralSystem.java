package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MineralSystem extends BaseHardware {

    DcMotor lifter, extendo, intake;

    Servo flipperServos;

    private static final double SERVO_LITTLE_POS = .25;
    private static final double SERVO_UP_POS = 1;
    private static final double SERVO_DOWN_POS = 0;

    private boolean servoSet = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        lifter = hardwareMap.dcMotor.get("lifter");
        intake = hardwareMap.dcMotor.get("intake");
        //extendo = hardwareMap.dcMotor.get("extendo");

        flipperServos = hardwareMap.servo.get("flippers");

        telemetry.addLine(getClass().toString() + " setup");
        telemetry.update();
    }

    public void controlSystem(Gamepad gamepad, Telemetry telemetry){

        if (gamepad.right_bumper) {
            flipperServos.setPosition(SERVO_UP_POS);
            servoSet = false;
        } else if (gamepad.left_bumper) {
            flipperServos.setPosition(SERVO_LITTLE_POS);
            servoSet = false;
        } else {
            if (!servoSet) {
                flipperServos.setPosition(SERVO_DOWN_POS);
                servoSet = true;
            }
        }

        //extendo.setPower(gamepad.left_stick_y);
        lifter.setPower(-gamepad.right_stick_y);

        intake.setPower(gamepad.left_trigger - gamepad.right_trigger);

        telemetry.addLine(flipperServos.getPosition() + "  ");

    }

    public void runIntake(double power){

        intake.setPower(power);

    }
}
