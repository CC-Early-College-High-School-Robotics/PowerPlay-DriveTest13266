
package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class VisionTest extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

     // Tag ID 1.2.3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        // Initialize OpenCvWebcam with live preview
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("OpenCV ran into an error", errorCode);
            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id ==RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("I see da tag\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Can't see the tag bro :/");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(Tag has not been seen yet)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Can't see the tag bro :/");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

//        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajectory
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
        } else{
            //trajectory

        }
            //trajectory

//        if(tagOfInterest == null)
//        {
//            /*
//             * Insert your autonomous code here, presumably running some default configuration
//             * since the tag was never sighted during INIT
//             */
//        }
//        else
//        {
//            /*
//             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
//             */
//
//            // e.g.
//            if(tagOfInterest.pose.x <= 20)
//            {
//                // do something
//            }
//            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
//            {
//                // do something else
//            }
//            else if(tagOfInterest.pose.x >= 50)
//            {
//                // do something else
//            }
//        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
       // while (opModeIsActive()) {sleep(20);}
        //add loop code
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addData("Detected tag ID", detection.id);
        telemetry.addData("Translation X in meters", detection.pose.x);
        telemetry.addData("Translation Y in meters", detection.pose.y);
        telemetry.addData("Translation Z in meters", detection.pose.z);
        telemetry.addData("Rotation Yaw in degrees", Math.toDegrees(detection.pose.yaw));
        telemetry.addData("Rotation Pitch degrees", Math.toDegrees(detection.pose.pitch));
        telemetry.addData("Rotation Roll degrees", Math.toDegrees(detection.pose.roll));
    }
}