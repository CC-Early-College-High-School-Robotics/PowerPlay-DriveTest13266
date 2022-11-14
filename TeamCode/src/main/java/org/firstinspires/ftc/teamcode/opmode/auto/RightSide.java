package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RightSide extends LinearOpMode {
    TrajectoryVelocityConstraint mediumSlowVel = (v, pose2d, pose2d1, pose2d2) -> 30; // value
    TrajectoryAccelerationConstraint mediumSlowAccel = (v, pose2d, pose2d1, pose2d2) -> 30; // value

    TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
    TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

    Pose2d startPose = new Pose2d(31, -62, Math.toRadians(90));

    double backDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        VisionSubsystem vision = new VisionSubsystem(this);
//        LiftSubsystem lift = new LiftSubsystem(this);
//        GripperSubsystem gripper = new GripperSubsystem(this);

        telemetry.setMsTransmissionInterval(50);
        drive.setPoseEstimate(startPose);
//        lift.initial();
//        gripper.close();

        vision.init();



        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
           telemetry.update();
           if (vision.getTagOfInterest() == null) continue;
           switch (vision.getTagOfInterest().id) {
               case 2: {
                   backDistance = 25;
                   break;
               }
               case 3: {
                   backDistance = 48;
                   break;
               }
               default: {
                   backDistance = 0;
               }
           }
        }
        waitForStart();

        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(20, -60), slowVel, slowAccel)
//                .run(lift::high)
                .splineToConstantHeading(new Vector2d(10, -47), Math.toRadians(90), slowVel, slowAccel)
                .splineTo(new Vector2d(4, -29), Math.toRadians(120), slowVel, slowAccel)
                .build();
        TrajectorySequence boxOne = drive.trajectorySequenceBuilder(preLoad.end())
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)), slowVel, slowAccel)
                .build();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(preLoad);
//        gripper.open();
        drive.followTrajectorySequence(boxOne);
//        lift.initial();
        if (backDistance == 0) return;
        TrajectorySequence forward = drive.trajectorySequenceBuilder(boxOne.end())
                .back(backDistance, slowVel, slowAccel)
                .build();
        drive.followTrajectorySequence(forward);
        sleep(5000);
    }
}
