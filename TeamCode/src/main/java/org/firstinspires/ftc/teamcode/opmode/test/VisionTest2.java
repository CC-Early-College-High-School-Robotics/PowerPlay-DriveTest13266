package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class VisionTest2  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem vision = new VisionSubsystem(this);

        vision.init();
        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
        }
        switch (vision.getTagOfInterest().id) {
            case 1: {

            }
            case 2: {

            }
            case 3: {

            }
        }
    }
}
