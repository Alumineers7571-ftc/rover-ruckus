package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DriveTypes.TANK;

@TeleOp(name = "good gyro 20 - offset 3", group = "Test")
public class IMUTesting extends LinearOpMode {

   Robot rb = new Robot();

   final int T1_ANGLE = 20;

   final int ANGLE_OFFSET = 3;

   double currentAngle;

   boolean done = false;

    @Override public void runOpMode() {

        rb.init(hardwareMap, telemetry, TANK);

        waitForStart();

        while (opModeIsActive() && !done) {

            currentAngle = rb.gyro.getGyroangle();

            rb.drive.turnAbsoulte(T1_ANGLE, currentAngle);
            currentAngle = (int)currentAngle;
            if (currentAngle >= T1_ANGLE - ANGLE_OFFSET && currentAngle <= T1_ANGLE + ANGLE_OFFSET) {
                rb.drive.setThrottle(0);
                done = false;
            }
        }
    }
}