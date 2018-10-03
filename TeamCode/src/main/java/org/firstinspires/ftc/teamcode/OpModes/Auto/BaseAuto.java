package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.ENUMS;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class BaseAuto extends OpMode {

    public Robot robot = new Robot();
    public SamplingOrderDetector detector;

    public ENUMS.CraterAutoStates robostate = null;

    //protected abstract void setup();
    //protected abstract void run();

    @Override
    public void init(){

        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        telemetry.addLine("robo init");
        telemetry.update();
    }

    @Override
    public void start() {

        robostate = ENUMS.CraterAutoStates.START;
    }

    @Override
    public void loop() {

        switch (robostate) {

            case START: {

                //init

                robostate = ENUMS.CraterAutoStates.FINDGOLD;
                break;
            }
            case FINDGOLD: {

                telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
                telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result

                //robostate = ENUMS.CraterAutoStates.FINDWALL
            }

        }

        telemetry.update();
    }

    @Override
    public void stop() {
        detector.disable();
    }

    public static void initAuto(ENUMS.AutoTypes autoTypes, Telemetry telemetry){

        telemetry.addLine("Yo, this: " + autoTypes.toString() + " es inited");
        telemetry.update();

    }


}
