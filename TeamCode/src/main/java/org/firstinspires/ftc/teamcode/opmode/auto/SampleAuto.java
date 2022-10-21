package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class SampleAuto extends LinearOpMode {
    TrajectoryVelocityConstraint vel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
    TrajectoryAccelerationConstraint accel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

    TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
    TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

    Pose2d startPose = new Pose2d(-40.3, -63.2, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        VisionSubsystem vision = new VisionSubsystem(this);

        telemetry.setMsTransmissionInterval(50);
        vision.init();
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
           telemetry.update();
        }

        double distance;
        switch (vision.getTagOfInterest().id) {
            case 2: {
                distance = 2;
            }
            case 3: {
                distance = 3;
            }
            default: {
                distance = 1;
            }
        }

        waitForStart();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .forward(distance)
                .build();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq);
    }
}
