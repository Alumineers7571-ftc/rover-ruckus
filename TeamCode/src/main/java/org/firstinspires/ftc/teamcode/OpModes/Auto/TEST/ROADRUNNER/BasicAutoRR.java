package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER.SampleMecanumDrive;

import java.io.IOException;

@Autonomous(name = "BasicAutoRR")
@Disabled
public class BasicAutoRR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory;

        TrajectoryConfig config = null;
        try {
            config = AssetsTrajectoryLoader.loadConfig("test10_27");
        } catch (IOException e) {
            telemetry.addData("BAD", "VBAD");
            telemetry.update();
            e.printStackTrace();
        }

        trajectory = config.toTrajectory();

        waitForStart();

        drive.followTrajectory(trajectory);

        while(opModeIsActive() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();

            drive.update();
            telemetry.addData("Angle: ", drive.getHeading());
            telemetry.addData("Wheel Pos: ", drive.getWheelPositions());
            telemetry.update();
        }

    }
}
