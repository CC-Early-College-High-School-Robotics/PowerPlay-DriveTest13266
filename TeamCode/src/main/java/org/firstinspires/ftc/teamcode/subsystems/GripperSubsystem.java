package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem.TrapdoorConstants.*;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class GripperSubsystem extends HardwareSubsystem {

    private final ServoEx trapdoor;

    public static class TrapdoorConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public String ID = "trapdoorServo";
            public double MIN_ANGLE = 0;
            public double MAX_ANGLE = 270;
            public boolean REVERSED = true;
        }
        public static class Value {
            public double OPEN                       = 9; // Degrees
            public double CLOSE                      = 90; // Degrees
        }
    }

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
