package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.TrapdoorConstants.hardware;
import static org.firstinspires.ftc.teamcode.constants.Constants.TrapdoorConstants.value;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class GripperSubsystem extends HardwareSubsystem {

    private final ServoEx trapdoor;

    public GripperSubsystem(OpMode opMode) {
        super(opMode);
        trapdoor = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MAX_ANGLE);
        trapdoor.setInverted(hardware.REVERSED);
    }

    public void open() {
        trapdoor.turnToAngle(value.OPEN);
    }
    public void close() {
        trapdoor.turnToAngle(value.CLOSE);
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }
}
