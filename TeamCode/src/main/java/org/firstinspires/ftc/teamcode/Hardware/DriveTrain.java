package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.Math7571;

import java.util.Locale;

public class DriveTrain extends BaseHardware {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;


    Telemetry telemetry;

    public DriveTrain() {
    }

    public enum DriveTypes{
        MECANUM, STANDARD, TANK, TANKWITHMECANUM, SIXWHEEL
    }

//---------------------------------------------------------------------------------------

    DriveTypes driveType;

    Math7571 mathEngine = new Math7571();

    boolean trSetUp = false;
    double wrappedAngle;
    double currentAngle;

    public DcMotor FL, FR, BL, BR;

    private BNO055IMU imu;

    public Orientation angles;

    private double gyroangle, startingValue;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveTypes type){
        this.initialize(hardwareMap, type, telemetry);
    }

//----------------------------------------------------------------------------------------

    private void initialize(HardwareMap hardwareMap, DriveTypes type, Telemetry telemetry){

        driveType = type;

        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        switch (driveType) {
            case MECANUM: {
                telemetry.addLine("MECANUM");
                break;
            }
            case STANDARD: {
                telemetry.addLine("STANDARD");
                break;
            }
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        startingValue = getGyroangle();

        imu.initialize(parameters);

        telemetry.addLine("DT & GYRO GOOD");
        telemetry.update();


    }

//------------------------------------------------------------------------------------------


    public void setThrottle(double power){

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

    }


    public void strafe(double power){

        //power > 0 = right
        //power < 0 = left

        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(-power);

    }

    public void runMotorsSides(double lPower, double rPower){

        FL.setPower(lPower);
        FR.setPower(rPower);
        BL.setPower(lPower);
        BR.setPower(rPower);

    }


    public void runMotorsIndiv(double flPower, double frPower, double blPower, double brPower){

        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);

    }

    //-----------------------------------------------------------------------------

    public void kill(){

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void resetUpEncoders(){

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void runMotorEncoders(int ticks){

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setTargetPosition(ticks);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void switchDirection(){
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void checkMotors(double power){

        telemetry.addLine("IsBusy?: " + FL.isBusy());
        telemetry.addLine("Mode?: " + FL.getMode() + FR.getMode() + BL.getMode() + BR.getMode());

        if (FL.isBusy()){
            FL.setPower(power);
            telemetry.addLine("settingPower");
        } else {
            FL.setPower(0);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("stopping");
        }

        telemetry.update();

    }

    public void setMotorModes(DcMotor.RunMode runMode){
        FL.setMode(runMode);
        FR.setMode(runMode);
        BL.setMode(runMode);
        BR.setMode(runMode);
    }

    //-----------------------------------------------------------------------------------------------------------------

    public void manualDrive(Gamepad gamepad){

        switch (driveType){

            case MECANUM:

                FL.setPower((-gamepad.left_stick_y) + gamepad.right_stick_x + gamepad.left_stick_x);
                FR.setPower((-gamepad.left_stick_y) - gamepad.right_stick_x - gamepad.left_stick_x);
                BL.setPower((-gamepad.left_stick_y) + gamepad.right_stick_x - gamepad.left_stick_x);
                BR.setPower((-gamepad.left_stick_y) - gamepad.right_stick_x + gamepad.left_stick_x);

                break;

            case TANKWITHMECANUM:

                if (gamepad.left_bumper){

                    strafe(1);

                } else if (gamepad.right_bumper){

                    strafe(-1);

                } else {

                    FL.setPower(gamepad.left_stick_y + gamepad.right_stick_x);
                    FR.setPower(gamepad.left_stick_y - gamepad.right_stick_x);
                    BL.setPower(gamepad.left_stick_y + gamepad.right_stick_x);
                    BR.setPower(gamepad.left_stick_y - gamepad.right_stick_x);

                }

            case TANK:

                FL.setPower(gamepad.left_stick_y + gamepad.right_stick_x);
                FR.setPower(gamepad.left_stick_y - gamepad.right_stick_x);
                BL.setPower(gamepad.left_stick_y + gamepad.right_stick_x);
                BR.setPower(gamepad.left_stick_y - gamepad.right_stick_x);
        }
    }


    public void turnAbsoulte(double target, double heading){

        double Error = heading - target;
        double Kp = 0.015;
        double leftPower;
        double rightPower;

        if ((Math.abs(Error)) > 2 ){
            leftPower = Error * Kp;
            rightPower = -Error * Kp;
            leftPower = Range.clip(leftPower,-1,1);
            rightPower = Range.clip(rightPower,-1,1);

            FL.setPower(leftPower);
            FR.setPower(rightPower);
            BL.setPower(leftPower);
            BR.setPower(rightPower);
        }
        else {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

    }

    public boolean turnRelative(double angle, double tolerence, LinearOpMode opMode) {

            if (!trSetUp) {

                wrappedAngle = mathEngine.wrapAround((int) (angle + getGyroangle()), -180, 180);
                trSetUp = true;

            }


            currentAngle = getGyroangle();

            turnAbsoulte(wrappedAngle, currentAngle);

            if ((currentAngle >= wrappedAngle-tolerence) && (currentAngle <= wrappedAngle+tolerence)) {
                setThrottle(0);
                wrappedAngle = 0;
                trSetUp = false;
                return false;
            }

            opMode.telemetry.addLine("angle: " + currentAngle);
            opMode.telemetry.addLine("wrapped: " + wrappedAngle);
            opMode.telemetry.addLine("setUp: " + trSetUp);
            opMode.telemetry.update();

            return true;

    }

    public void adjustHeading(int targetHeading, boolean slow) {
        //Initialize the turnleft boolean.
        boolean turnLeft = false;
        double leftPower = 0, rightPower = 0;

        //Get the current heading from the imu.
        float curHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //If within a reasonable degree of error of the target heading, set power to zero on all motors.
        if (Math.abs(curHeading - targetHeading) <= 1) {
            setThrottle(0);
            return;
        }
        //Generate our proportional term
        float powFactor = Math.abs(targetHeading - curHeading) * (float) (slow ? .0055 : .02);

        //Choose the direction of the turn based on given target and current heading
        switch (targetHeading) {
            case 0:
                turnLeft = curHeading <= 0;
                break;

            case 90:
                turnLeft = !(curHeading <= -90 || curHeading >= 90);
                break;

            case 180:
                turnLeft = !(curHeading <= 0);
                break;

            case -90:
                turnLeft = curHeading <= -90 || curHeading >= 90;
                break;

            case 45:
                turnLeft = !(curHeading <= -135 || curHeading >= 45);
                break;

            case -45:
                turnLeft = curHeading <= -45 || curHeading >= 45;
                break;

            case 30:
                turnLeft = !(curHeading <= -30 || curHeading >= 30);
                break;

            case -30:
                turnLeft = curHeading <= -30 || curHeading >= 30;
                break;

            case 60:
                turnLeft = !(curHeading <= -120 || curHeading >= 60);
                break;

            case -60:
                turnLeft = curHeading <= -60 || curHeading >= 60;
                break;

            default:
                turnLeft = targetHeading < 0 ? (curHeading <= targetHeading || curHeading >= -1*targetHeading) : !(curHeading <= -1*targetHeading || curHeading >= targetHeading);
        }

        //Clip the powers to within an acceptable range for the motors and apply the proportional factor.
        leftPower = Range.clip((turnLeft ? -1 : 1) * powFactor, -1, 1);
        rightPower = Range.clip((turnLeft ? 1 : -1) * powFactor, -1, 1);

        //Set power to all motors
        FL.setPower(leftPower);
        BL.setPower(leftPower);
        FR.setPower(rightPower);
        BR.setPower(rightPower);

    }

    public void adjustHeadingRelative(int targetHeading, boolean slow){

        targetHeading += getGyroangle();

        adjustHeading(targetHeading, slow);


    }



    public void moveEncoder(int inchesLeft, int inchesRight, double speed){

        int lfPose = FL.getCurrentPosition() + (int)(inchesLeft * COUNTS_PER_INCH);
        int lrPose = BL.getCurrentPosition() + (int)(inchesLeft * COUNTS_PER_INCH);
        int rfPos = FR.getCurrentPosition() + (int)(inchesRight * COUNTS_PER_INCH);
        int rrPos = BR.getCurrentPosition() + (int)(inchesRight * COUNTS_PER_INCH);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setTargetPosition(lfPose);
        BL.setTargetPosition(lrPose);
        FR.setTargetPosition(rfPos);
        BR.setTargetPosition(rrPos);

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);
    }

    public boolean motorsBusy(){
        return FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy();
    }

    public double getGyroangle(){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        return gyroangle;

    }

    public double getStartingValue(){
        return startingValue;
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public boolean calibrateGyro(){

        return isCalib();
    }

    public boolean isCalib(){
        return imu.isGyroCalibrated();
    }

}
