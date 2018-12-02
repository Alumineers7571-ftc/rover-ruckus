package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Control.ENUMS;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.Control.FinalValues.goldPosMissedMostOften;

@Autonomous (name = "Depot", group = "MAIN")
public class Depot extends LinearOpMode {

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

    BNO055IMU imu;

    Robot rb = new Robot();

    //final int NAVTOGOLD_ONE_TURN_ANGLE = 50;
    //final int NAVTOGOLD_TWO_TURN_ANGLE = -140;

    //final int SAMPLE_TURN_ANGLE = 90;

    final int ANGLE_OFFSET = 5;

    //int count = 0;

    boolean goldFound = false;

    double currentAngle;

    boolean NAV1_TURN_DONE = false, NAV2_TURN_DONE = false, SAMPLE_TURN_DONE = false;

    ENUMS.GoldPosition goldPosition = ENUMS.GoldPosition.UNKNOWN;

    ENUMS.AutoStates robo = ENUMS.AutoStates.START;

    long delayTime = 0;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .6, correction;

    int startingAngle = 0;
    
    PIDController pidRotate;
    
    @Override
    public void runOpMode() {

        rb.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK,false);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 20 degrees of rotation.
        pidRotate = new PIDController(.002, 0, 0);


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();



        pidRotate = new PIDController(.002, 0, 0);
        
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

        vuforiaParameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        vuforiaParameters.fillCameraMonitorViewParent = true;

        vuforiaParameters.cameraName = webcamName;

        vuforia = new Dogeforia(vuforiaParameters);
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
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        rb.tm.setTMUp();

        //startingAngle = (int)rb.drive.getGyroangle();

        telemetry.addLine("Ready");

        while(!isStarted()){

            if(gamepad1.y){
                delayTime += 1000;
            } else if (gamepad1.x){
                if (delayTime > 0) {
                    delayTime -= 1000;
                }
            }

            if(gamepad1.dpad_up){
                power += 0.1;
            } else if(gamepad1.dpad_down){
                power -= 0.1;
            } else if(gamepad1.dpad_left){
                power = 0;
            } else if(gamepad1.dpad_right){
                power = 0.4;
            }

            /*if(detector.isFound()){
                detector.disable();
                goldFound = true;
                goldPosition = ENUMS.GoldPosition.CENTER;
                robo = ENUMS.AutoStates.EARLYCENTERGOLD;
            }*/

            telemetry.addData("delayTime", delayTime);
            telemetry.addData("turnPower ", power);
            telemetry.addData("gold", detector.isFound());
            telemetry.addData("robo", robo);
            telemetry.addData("cangle", getAngle());
            telemetry.update();

            sleep(1000);

        }

        waitForStart();

        sleep(delayTime);

        while (opModeIsActive() && !isStopRequested()) {

            switch (robo) {

                case START: {

                    robo = ENUMS.AutoStates.DROPDOWN;
                    break;
                }

                case DROPDOWN: {

                    while(opModeIsActive() && !rb.hanger.moveToGround()){

                    }

                    sleep(300);
                    imu.initialize(parameters);

                    // make sure the imu gyro is calibrated before continuing.
                    while (!isStopRequested() && !imu.isGyroCalibrated()) {
                        sleep(50);
                        idle();
                    }

                    rb.drive.strafe(0.3);
                    sleep(1000);
                    rb.drive.setThrottle(0);
                    sleep(100);
                    rb.drive.encoderDrive(0.4, 2, 2, opModeIsActive());
                    sleep(100);
                    rb.drive.strafe(-0.3);
                    sleep(1000);
                    rb.drive.setThrottle(0);

                    robo = ENUMS.AutoStates.MOVETOSAMPLE;
                    break;

                }

                case MOVETOSAMPLE: {

                    rb.drive.encoderDrive(0.6, 17, 17, opModeIsActive());

                    sleep(500);

                    rotate(30, power);

                    robo = ENUMS.AutoStates.CHECKLEFT;
                    telemetry.update();
                    break;
                }

                case CHECKLEFT: {

                    if (detector.isFound()) {
                        detector.disable();
                        goldFound = true;
                        goldPosition = ENUMS.GoldPosition.LEFT;
                        robo = ENUMS.AutoStates.HITGOLD;
                    } else {

                        robo = ENUMS.AutoStates.CHECKCENTER;

                    }

                    telemetry.update();
                    break;
                }

                case CHECKCENTER:{

                    rotate(-30, power);
                    sleep(500);

                    if(detector.isFound()){
                        detector.disable();
                        goldFound = true;
                        goldPosition = ENUMS.GoldPosition.CENTER;
                        robo = ENUMS.AutoStates.HITGOLD;
                    } else {
                        robo = ENUMS.AutoStates.CHECKRIGHT;
                        break;
                    }

                    telemetry.update();
                    break;
                }

                case CHECKRIGHT: {

                    rotate(-30, power);
                    sleep(500);

                    if (detector.isFound()) {
                        detector.disable();
                        goldFound = true;
                        goldPosition = ENUMS.GoldPosition.RIGHT;
                    }

                    robo = ENUMS.AutoStates.HITGOLD;
                    telemetry.update();
                    break;
                }

                case EARLYCENTERGOLD:{

                    rb.drive.setThrottle(0.6);
                    sleep(3000);

                    robo = ENUMS.AutoStates.DROPTM;
                    telemetry.update();
                    break;
                }

                case HITGOLD: {

                    //rb.mineralSystem.runIntake(1);

                    if (goldPosition == ENUMS.GoldPosition.LEFT){

                        float targetPos = (float) getAngle();

                        rotate(-30, power);

                        sleep(500);
                        rb.drive.strafe(0.5);
                        sleep(1000);
                        rb.drive.setThrottle(0);
                        sleep(300);
                        rb.drive.setThrottle(0.4);
                        sleep(2000);
                        rb.drive.setThrottle(0);

                    } else if (goldPosition == ENUMS.GoldPosition.CENTER) {

                        rb.drive.setThrottle(0.4);
                        sleep(3000);
                        rb.drive.setThrottle(0);

                    } else if (goldPosition == ENUMS.GoldPosition.RIGHT){

                        float targetPos = (float) getAngle();

                        rotate(30, power);

                        sleep(500);

                        rb.drive.strafe(-0.5);
                        sleep(1000);
                        rb.drive.setThrottle(0);
                        sleep(300);
                        rb.drive.setThrottle(0.4);
                        sleep(2000);
                        rb.drive.setThrottle(0);

                        robo = ENUMS.AutoStates.FINDWALLFORDEPOT;
                        telemetry.update();

                    } else if (goldPosition == ENUMS.GoldPosition.UNKNOWN){

                        goldPosition = goldPosMissedMostOften;

                        //maybe check again?

                    }

                    //rb.mineralSystem.runIntake(0);

                    if(goldFound) {
                        robo = ENUMS.AutoStates.FINDWALLFORDEPOT;
                        telemetry.update();
                    }

                    telemetry.update();
                    break;
                }

                case FINDWALLFORDEPOT:{

                    if(goldPosition == ENUMS.GoldPosition.RIGHT) {
                        rotate(40, power);
                        sleep(200);
                        rb.drive.strafe(-0.5);
                        sleep(1000);
                        rb.drive.setThrottle(0);
                    } else if (goldPosition == ENUMS.GoldPosition.LEFT) {
                        rotate(-40, power);
                        sleep(200);
                        rb.drive.strafe(0.5);
                        sleep(2000);
                        rb.drive.setThrottle(0);
                    }

                    sleep(300);
                    rb.drive.setThrottle(0.6);
                    sleep(1500);
                    rb.drive.setThrottle(0);

                    robo = ENUMS.AutoStates.DROPTM;
                    break;
                }

                case DROPTM:{

                    sleep(200);
                    rb.tm.setTMDown();
                    sleep(300);
                    rb.drive.setThrottle(-8);
                    sleep(1000);
                    rb.tm.setTMUp();
                    robo = ENUMS.AutoStates.FINDCRATER;
                    break;

                }

                case FINDCRATER:{

                    sleep(750);
                    rb.drive.setThrottle(0);

                    robo = ENUMS.AutoStates.END;
                    telemetry.update();
                    break;
                }

                case END: {
                    stop();
                }
            }

            telemetry.addData("step", robo);
            telemetry.addData("current angle", getAngle());
            telemetry.addData("gold pos", goldPosition);
            telemetry.addData("gold?", goldFound);
            telemetry.update();

            idle();
        }

    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // rb.drive.getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                rb.drive.runMotorsSides(power, -power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                rb.drive.runMotorsSides(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                rb.drive.runMotorsSides(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rb.drive.setThrottle(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    
}