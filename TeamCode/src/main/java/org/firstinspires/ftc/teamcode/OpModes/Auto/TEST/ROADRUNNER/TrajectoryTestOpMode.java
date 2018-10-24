package org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Auto.TEST.ROADRUNNER.SampleMecanumDrive;

/*
 * This is a simple routine to test trajectory following capabilities. It consists of two 180deg
 * turns, one straight line, and a spline. It is highly recommended to try this **without feedback**
 * to ensure that all of the other drive constants are configured properly. If there are issues,
 * please debug them and retune instead of relying on feedback. The entire goal of feedforward is to
 * minimize the need for feedback. Once the feedforward is working, begin adding feedback control
 * (tune velocity first).
 */
@Autonomous
public class TrajectoryTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // change these constraints to something reasonable for your drive
        Trajectory trajectory = drive.trajectoryBuilder()
                .turnTo(Math.PI)
                .waitFor(2)
                .turnTo(0)
                .waitFor(2)
                .lineTo(new Vector2d(60, 0))
                .waitFor(2)
                .splineTo(new Pose2d(0, 40, 0))
                .build();

        waitForStart();

        drive.followTrajectory(trajectory);
        while (opModeIsActive() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();

            drive.update();
        }
    }
}
