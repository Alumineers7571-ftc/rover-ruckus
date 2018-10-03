package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Deprecated
public class GlyphSystem extends BaseHardware{

    DcMotor intakeLeft, intakeRight, lifter, flipper;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        lifter = hardwareMap.dcMotor.get("lifter");
        flipper = hardwareMap.dcMotor.get("flipper");

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("GlyphSystem setup");
        telemetry.update();

    }

    //-----------------------------------------------------------------------------

    public void runIntake(double power){

        intakeLeft.setPower(power);
        intakeRight.setPower(power);

    }

    public void runLifter(double power){

        lifter.setPower(power);

    }

    public void runFlipper(double power){

        flipper.setPower(power);

    }

    //-------------------------------------------------------------------------------

    public void controlSystem(Gamepad gamepad){

        if (gamepad.right_bumper){

            runIntake(1);

        } else if (gamepad.left_bumper){

            runIntake(-1);

        } else {

            runIntake(0);

        }

        runLifter(gamepad.right_stick_y);

        runFlipper(gamepad.right_trigger - gamepad.left_trigger);


    }






}
