package org.firstinspires.ftc.teamcode.opmode.auto.position1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous
public class position1 extends LinearOpMode {
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
                .splineToConstantHeading(new Vector2d(13, -47), Math.toRadians(90))
                .splineTo(new Vector2d(8, -32), Math.toRadians(134))
                .build()
        );
    }
}
