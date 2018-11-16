package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN.Old;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Control.ENUMS;
import org.firstinspires.ftc.teamcode.Control.FinalValues;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Gyro;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous (name = "BlueCrater", group = "MAIN")
@Disabled
public class BlueCrater extends LinearOpMode {

    private Robot robot = new Robot();
    private SamplingOrderDetector detector;

    ENUMS.AutoStates robostate = null;
    SamplingOrderDetector.GoldLocation goldLocation = null;

    public double currentAngle;
    public double currentAngleOffset = 0;

    //protected abstract void setup();
    //protected abstract void run();

    @Override
    public void runOpMode(){

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

        waitForStart();

        robostate = ENUMS.AutoStates.START;

        while(opModeIsActive()) {


            //telemetry.addData("GyroAngle: ", robot.gyro.gyroangle);
            //telemetry.addData("Angle: ", robot.gyro.angles);

            //currentAngle = robot.gyro.getGyroangle();

            switch (robostate) {

                case START: {

                    //init

                    robostate = ENUMS.AutoStates.FINDGOLD;
                    break;
                }
                case FINDGOLD: {

                    telemetry.addData("Current Order", detector.getCurrentOrder().toString()); // The current result for the frame
                    telemetry.addData("Last Order", detector.getLastOrder().toString()); // The last known result

                    goldLocation = detector.getLastOrder();

                    robostate = ENUMS.AutoStates.TESTGYRO;
                    break;
                }
                case TESTGYRO: {

                   // robot.drive.turnAbsoulte(90, currentAngle);
                    if (currentAngle >= 86 && currentAngle <= 94){
                        robot.drive.setThrottle(0);
                        currentAngleOffset = currentAngle;

                        robostate = ENUMS.AutoStates.FINDWALL;

                        break;
                    }

                }
                case HITGOLD: {

                    if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {
                        //robot.drive.turnAbsoulte(20, currentAngle);
                        if (currentAngle <= 18 && currentAngle >= 22){
                            robot.drive.setThrottle(0);
                            robot.drive.kill();
                            currentAngleOffset = currentAngle;
                        }
                    } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {
                      //  robot.drive.turnAbsoulte(-20, currentAngle);
                        if (currentAngle <= -18 && currentAngle >= -22){
                            robot.drive.setThrottle(0);
                            robot.drive.kill();
                            currentAngleOffset = currentAngle;
                        }
                    }

                    robot.drive.setThrottle(0.2);
                    //mineralSystem.runIntake(1);
                    sleep(1000);
                    robot.drive.setThrottle(0);
                    sleep(1000);
                    //mineralSystem.runIntake(0);

                    detector.disable();
                    break;
                }
                case FINDWALL: {

                    robot.inputManager.setupPlayback(FinalValues.cbSampleToTM);
                    robot.inputManager.replayInputs();

                    break;

                }



            }

            telemetry.update();
        }

    }
}
