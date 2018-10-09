package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.CV;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

@Autonomous (name = "WebcamGold")
public class WebCamGold extends LinearOpMode {

    GoldAlignDetector detector;

    DriveTrain dt = new DriveTrain();

    @Override
    public void runOpMode(){

        dt.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);

        detector = new GoldAlignDetector();
        //detector.init(hardwareMap.get(WebcamName.class, "Webcam 1"), CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

            if (detector.getXPosition() > 3.0){
                dt.setThrottle(0.2);
            } else {
                dt.setThrottle(0);
            }

        }

    }


}
