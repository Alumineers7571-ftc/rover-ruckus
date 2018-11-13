package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.FinalValues;
import org.firstinspires.ftc.teamcode.Hardware.Recording.InputManager;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "Start To TM", group = "test")
@Disabled
public class StartToTM extends LinearOpMode{

    private Robot robot = new Robot();

    private int roboState = 0;

    @Override
    public void runOpMode(){

        telemetry.addLine(roboState + " yeah");

        robot.inputManager.init(hardwareMap, telemetry);

        waitForStart();

        robot.inputManager.setupPlayback(FinalValues.cbStartToTM);

        roboState++;

        robot.inputManager.replayInputs();

        robot.inputManager.stopAndReset();

        roboState++;

        robot.inputManager.setupPlayback(FinalValues.cbTMtoCrater);

        roboState++;

        robot.inputManager.replayInputs();

        roboState++;

    }

}
