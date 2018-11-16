package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN.Scrimmage;


import android.content.Context;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Control.ENUMS;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Gyro;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous (name = "DepotScrim")
@Disabled
public class DepotScrim extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    GoldAlignDetector detector;
    
    Robot rb = new Robot();

    final int NAVTOGOLD_ONE_TURN_ANGLE = 50;
    final int NAVTOGOLD_TWO_TURN_ANGLE = -140;

    final int SAMPLE_TURN_ANGLE = 90;

    final int PRECISE_ANGLE_OFFSET = 5;
    final int GENERAL_ANGLE_OFFSET = 5;

    double currentAngle;

    boolean NAV1_TURN_DONE = false, NAV2_TURN_DONE = false, SAMPLE_TURN_DONE = false;

    ENUMS.AutoStates robo = ENUMS.AutoStates.NAVTOSAMPLE1;

    @Override
    public void runOpMode() {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamName;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        //--------------------------------------------------------------------------------------------------
        // CHANGE these when you can
        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        //--------------------------------------------------------------------------------------------------

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        rb.tm.setTMUp();

        telemetry.addData("Ready", "true");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            switch (robo) {

                case START: {

                    robo = ENUMS.AutoStates.NAVTOSAMPLE1;
                    break;
                }

                case NAVTOSAMPLE1:{

                    rb.drive.adjustHeading(NAVTOGOLD_ONE_TURN_ANGLE, true);

                    robo = ENUMS.AutoStates.MOVETOTURN2;

                    break;
                }

                case MOVETOTURN2:{

                    currentAngle = 0;

                    sleep(300);
                    rb.drive.setThrottle(0.4);
                    sleep(2000);
                    rb.drive.setThrottle(0);
                    sleep(300);

                    robo = ENUMS.AutoStates.NAVTOSAMPLE2;
                    break;

                }

                case NAVTOSAMPLE2:{

                    rb.drive.adjustHeading(NAVTOGOLD_TWO_TURN_ANGLE, true);

                    robo = ENUMS.AutoStates.FINDGOLD;

                    break;

                }

                case FINDGOLD: {
                    telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
                    telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.

                    if (detector.isFound()) {
                        rb.drive.setThrottle(0);
                        detector.disable();
                        robo = ENUMS.AutoStates.TURNTOGOLD;
                    } else {
                        rb.drive.setThrottle(0.3);

                    }

                    break;
                }

                case TURNTOGOLD: {

                    detector.disable();

                    rb.drive.adjustHeading(SAMPLE_TURN_ANGLE, true);

                    break;
                }

                case HITGOLD: {
                    rb.drive.setThrottle(0.5);
                    sleep(1500);
                    rb.drive.setThrottle(0);
                    robo = ENUMS.AutoStates.DROPTM;
                    break;
                }

                case DROPTM:{

                    sleep(200);
                    rb.tm.setTMDown();
                    sleep(100);
                    rb.drive.setThrottle(-0.8);
                    sleep(1000);
                    rb.drive.setThrottle(0);

                    robo = ENUMS.AutoStates.FINDCRATER;
                    break;

                }

                case FINDCRATER:{

                }

                case END: {
                    stop();
                }
            }
        }

    }
}