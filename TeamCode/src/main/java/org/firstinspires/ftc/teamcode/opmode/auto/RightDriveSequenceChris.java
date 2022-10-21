package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightDriveSequenceChris extends LinearOpMode {
    Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        LiftSubsystem lift = new LiftSubsystem(this);
        GripperSubsystem claw = new GripperSubsystem(this);



        drive.setPoseEstimate(startPose);
        class RetardedPipelineshit extends LinearOpMode {
            OpenCvCamera camera;
            AprilTagDetectionPipeline aprilTagDetectionPipeline;

            static final double FEET_PER_METER = 3.28084;

            // Lens intrinsics
            // UNITS ARE PIXELS
            // NOTE: this calibration is for the C920 webcam at 800x448.
            // You will need to do your own calibration for other configurations!
            final double fx = 578.272;
            final double fy = 578.272;
            final double cx = 402.145;
            final double cy = 221.506;

            // UNITS ARE METERS
            final double tagsize = 0.166; // meters

            // Tag ID 1.2.3 from the 36h11 family
            final int LEFT = 1;
            final int MIDDLE = 2;
            final int RIGHT = 3;

            final AprilTagDetection tagOfInterest = null;

            @Override
            public void runOpMode() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

                camera.setPipeline(aprilTagDetectionPipeline);
                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                });

                telemetry.setMsTransmissionInterval(50);

                //open cv shit

                if (tagOfInterest == null || tagOfInterest.id == LEFT) {

                } else if (tagOfInterest.id == MIDDLE) {
                    //trajectory
                } else if (tagOfInterest.id == RIGHT) {
                    //trajectory
                }
          claw.close();
          lift.initial();


                TrajectorySequence Hello = drive.trajectorySequenceBuilder(startPose)
                        .lineToConstantHeading(new Vector2d(20, -60))
                        .run(lift::high)
                        .lineToLinearHeading(new Pose2d(8, -32, Math.toRadians(315)))
                        .back(4)
                        .run(claw::open)
                        .forward(4)
                        .run(lift::initial)
                        .lineToLinearHeading(new Pose2d(24, -36, Math.toRadians(180)))
                        .back(4)
                        .run(claw::close)
                        .forward(4)
                        .lineToLinearHeading(new Pose2d(8, -32, Math.toRadians(315)))
                        .run(lift::high)
                        .back(4)
                        .run(claw::open)
                        .forward(4)
                        .lineToLinearHeading(new Pose2d(24, -36, Math.toRadians(180)))
                        //depending on dectection determines distance going forward
                        .forward(10)
                        .run(lift::initial)
                        .build();

                waitForStart();

                if (!isStopRequested())
                    drive.followTrajectorySequence(Hello);
            }


        }
    }
}