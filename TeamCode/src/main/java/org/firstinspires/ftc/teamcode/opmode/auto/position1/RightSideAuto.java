package org.firstinspires.ftc.teamcode.opmode.auto.position1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RightSideAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(35, -62, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        // Initalize Subsystems
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        /** On initialization **/
        drive.setPoseEstimate(startPose);
        /** Open CV **/

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(20, -60))
                .lineToLinearHeading(new Pose2d(8, -32, Math.toRadians(315)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(24, -36, Math.toRadians(180)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(8, -32, Math.toRadians(315)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(24,-36, Math.toRadians(180)))
                .forward(10)
                .build()
        );
    }
}
