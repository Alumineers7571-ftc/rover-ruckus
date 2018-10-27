package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER.SampleMecanumDrive;

import java.io.IOException;

public class BasicAutoRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory;

        TrajectoryConfig config = null;
        try {
            config = AssetsTrajectoryLoader.loadConfig("test10_26");
        } catch (IOException e) {
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
