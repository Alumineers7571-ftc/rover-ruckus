/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Hardware.Gyro;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Double Sample V1", group="Double Sample")

public class DoubleSample extends LinearOpMode {

    // Declare OpMode members.

    private GoldAlignDetector detector;

    private Gyro gyro;

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private Servo servo;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gyro.init(hardwareMap, telemetry);

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        //--------------------------------------------------------------------------------------------------
        // CHANGE these when you can
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        //--------------------------------------------------------------------------------------------------

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.downscale = 0.3;
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 50;
        detector.alignPosOffset = 200;

        detector.enable();

        leftFront = hardwareMap.dcMotor.get("fl");
        leftRear = hardwareMap.dcMotor.get("bl");
        rightFront = hardwareMap.dcMotor.get("fr");
        rightRear = hardwareMap.dcMotor.get("br");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        waitForStart();

        //
        // AUTO START
        //
        moveEncoder(200, -200, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(1000, 1000, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-720, 720, 0.3);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-800, -800, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        int tickRef = leftFront.getCurrentPosition();
        moveEncoder(2500, 2500, 0.2);

        while(!detector.getAligned() && motorsBusy() && !isStopRequested()){
            telemetry.addData("Aligned", detector.getXPosition());
            telemetry.update();
        }
        int ticksLeft = 2500 -( leftFront.getCurrentPosition() - tickRef);
        setAllMotors(0);

        moveEncoder(500, -500, 0.3 );
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(400 , 400, 0.6);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-300 , -300, 0.6);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-500, 500, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(ticksLeft, ticksLeft, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-530, 530, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(2000, 2000, 0.2);

        while(!detector.getAligned() && motorsBusy() && !isStopRequested()){
            telemetry.addData("Aligned", detector.getXPosition());
            telemetry.update();
        }

        setAllMotors(0);

        moveEncoder(500, -500, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(1000, 1000, 0.9);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        servo.setPosition(-0.8);

        servo.setPosition(0.5);


        moveEncoder(700, -700, 0.4);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(1000, 1000, 0.2);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(150, -150, 0.2);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(4000, 4000, 1.0);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);


        detector.disable();

    }

    private void moveEncoder(int ticksLeft, int ticksRight, double speed){
        int lfPose = leftFront.getCurrentPosition() + ticksLeft;
        int lrPose = leftRear.getCurrentPosition() + ticksLeft;
        int rfPos = rightFront.getCurrentPosition() + ticksRight;
        int rrPos = rightRear.getCurrentPosition() + ticksRight;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(lfPose);
        leftRear.setTargetPosition(lrPose);
        rightFront.setTargetPosition(rfPos);
        rightRear.setTargetPosition(rrPos);

        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
    }

    private boolean motorsBusy(){
        return leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy();
    }

    private void setAllMotors(double speed){
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
    }

    private void turnToAngle(double angle, double speed){

    }
}
