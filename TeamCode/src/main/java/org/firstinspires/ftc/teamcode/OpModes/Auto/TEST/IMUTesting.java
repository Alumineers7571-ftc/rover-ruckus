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
import org.firstinspires.ftc.teamcode.Control.Math7571;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Gyro;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DriveTypes.TANK;

@TeleOp(name = "good gyro", group = "Test")
public class IMUTesting extends LinearOpMode {

   //Robot rb = new Robot();

    DriveTrain drive = new DriveTrain();
    Gyro gyro = new Gyro();
    Math7571 mathEngine = new Math7571();

   int T1_ANGLE;

   int TOlERANCE = 2;

   double currentAngle, startingAngle;

   boolean done = false;

    @Override public void runOpMode() {

        drive.init(hardwareMap, telemetry, TANK);
        gyro.init(hardwareMap, telemetry);

        while(!gyro.isCalib() && !isStopRequested()){}

        telemetry.addLine("READY");
        telemetry.update();

        startingAngle = gyro.getGyroangle();

        T1_ANGLE = mathEngine.wrapAround((int)(90 + startingAngle), -180, 180);

        waitForStart();

        while (opModeIsActive() && !done) {

            currentAngle = gyro.getGyroangle();

            drive.turnAbsoulte(T1_ANGLE, currentAngle);

            if (Math.abs(currentAngle - 90) < TOlERANCE) {
                drive.setThrottle(0);
                done = true;
            }

            telemetry.addLine("Angle: " + currentAngle);
            telemetry.addLine("T1_ANGLE: " + T1_ANGLE);
            telemetry.addLine("Starting Angle" + startingAngle);
            telemetry.update();

        }
    }
}