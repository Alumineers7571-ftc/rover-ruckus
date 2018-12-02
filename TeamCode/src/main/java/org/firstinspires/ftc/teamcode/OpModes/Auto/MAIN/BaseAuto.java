package org.firstinspires.ftc.teamcode.OpModes.Auto.MAIN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class BaseAuto {

    Robot rb;
    LinearOpMode opMode;

    PIDController pidRotate;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .6, correction;

    public BaseAuto(Robot robot, LinearOpMode opm){
        this.rb = robot;
        this.opMode = opm;
    }

    public void SetUp(){

        pidRotate = new PIDController(.002, 0, 0);

    }


    public void DropDown(){

        while(opMode.opModeIsActive() && rb.hanger.moveToGround()){
        }

        rb.drive.initIMU();

        while (!opMode.isStopRequested() && !rb.drive.isCalib()) {
            opMode.sleep(50);
            opMode.idle();
        }

        opMode.sleep(300);

        rb.drive.strafe(0.3);
        opMode.sleep(1000);
        rb.drive.setThrottle(0);
        opMode.sleep(100);
        rb.drive.encoderDrive(0.4, 2, 2, opMode.opModeIsActive());
        opMode.sleep(100);
        rb.drive.strafe(-0.3);
        opMode.sleep(1000);
        rb.drive.setThrottle(0);

    }

    public void MoveToSample(){

        rb.drive.encoderDrive(0.6, 17, 17, opMode.opModeIsActive());

        opMode.sleep(500);

        rotate(30, power);

    }

//-------------------------------------------------------------------------------------------------------------
    //SUB METHODS
//-------------------------------------------------------------------------------------------------------------



    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = rb.drive.getIMUOrientation();

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
        lastAngles = rb.drive.getIMUOrientation();

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
            while (opMode.opModeIsActive() && getAngle() == 0) {
                rb.drive.runMotorsSides(power, -power);
                opMode.sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                rb.drive.runMotorsSides(-power, power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                rb.drive.runMotorsSides(-power, power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rb.drive.setThrottle(0);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


}
