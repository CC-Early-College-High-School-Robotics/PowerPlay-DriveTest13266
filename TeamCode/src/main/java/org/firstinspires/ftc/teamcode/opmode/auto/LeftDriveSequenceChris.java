package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftDriveSequenceChris extends LinearOpMode {
    Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(-90));
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        LiftSubsystem lift = new LiftSubsystem();
        GripperSubsystem claw = new GripperSubsystem();


        Pose2d StartPose = new Pose2d(-35, -62, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Hello = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-20, -60))
                .lineToLinearHeading(new Pose2d(-8, -32, Math.toRadians(225)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(180)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(-8, -32, Math.toRadians(225)))
                .back(4)
                .forward(4)
                .lineToLinearHeading(new Pose2d(-24,-36, Math.toRadians(180)))
                .forward(10)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(Hello);
    }
}