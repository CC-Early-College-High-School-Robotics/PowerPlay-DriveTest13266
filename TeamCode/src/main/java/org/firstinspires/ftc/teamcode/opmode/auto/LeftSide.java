package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftSide extends LinearOpMode {
    TrajectoryVelocityConstraint mediumSlowVel = (v, pose2d, pose2d1, pose2d2) -> 30; // value
    TrajectoryAccelerationConstraint mediumSlowAccel = (v, pose2d, pose2d1, pose2d2) -> 30; // value

    TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
    TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

    Pose2d startPose = new Pose2d(-40, -62, Math.toRadians(-90));

    double forwardDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        VisionSubsystem vision = new VisionSubsystem(this);
        LiftSubsystem lift = new LiftSubsystem(this);
        GripperSubsystem gripper = new GripperSubsystem(this);

        telemetry.setMsTransmissionInterval(50);
        drive.setPoseEstimate(startPose);
        lift.initial();
        gripper.close();

        vision.init();



        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
           telemetry.update();
           if (vision.getTagOfInterest() == null) continue;
           switch (vision.getTagOfInterest().id) {
               case 2: {
                   forwardDistance = 24;
                   break;
               }
               case 3: {
                   forwardDistance = 48;
                   break;
               }
               default: {
                   forwardDistance = 0;
               }
           }
        }
        waitForStart();

        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-20, -60), slowVel, slowAccel)
                .splineToConstantHeading(new Vector2d(-13, -47), Math.toRadians(90), slowVel, slowAccel)
                .run(lift::high)
                .splineTo(new Vector2d(-8, -32), Math.toRadians(45), slowVel, slowAccel)
                .build();
        TrajectorySequence boxOne = drive.trajectorySequenceBuilder(preLoad.end())
                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(180)), slowVel, slowAccel)
                .build();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(preLoad);
        gripper.open();
        drive.followTrajectorySequence(boxOne);
        lift.initial();
        if (forwardDistance == 0) return;
        TrajectorySequence forward = drive.trajectorySequenceBuilder(boxOne.end())
                .forward(forwardDistance, slowVel, slowAccel)
                .build();
        drive.followTrajectorySequence(forward);
    }
}
