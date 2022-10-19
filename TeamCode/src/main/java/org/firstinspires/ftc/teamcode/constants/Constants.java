package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // TODO: remove all statics except for utility classes so that we can have folders
   // public static ArmConstants armConstants;
    public static CameraConstants cameraConstants;
    public static DriveConstants driveConstants;
    public static TrapdoorConstants trapdoorConstants;
    public static LiftServoConstants liftServoConstants;
    public static LiftMotorConstants liftMotorConstants;

    public static class CameraConstants {
        public static Hardware hardware = new Hardware();
            public static Value value = new Value();

        public static class Hardware {
            public boolean REVERSED = false;

        }
        public static class Value {
            public double
                    BLUE_WAREHOUSE             = 1, // Degrees
                    BLUE_CAROUSEL              = 1, // Degrees
                    RED_WAREHOUSE              = 1, // Degrees
                    RED_CAROUSEL               = 1; // Degrees
            public double CAMERA_WAIT_TIME_DOUBLE = 5;
            public long CAMERA_WAIT_TIME = (long) (CAMERA_WAIT_TIME_DOUBLE * 1000);
        }
    }

    public static class LiftMotorConstants {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String LEFT_ID = "leftLiftMotor";
            public String RIGHT_ID = "rightLiftMotor";
            public boolean LEFT_REVERSED = true;
            public boolean RIGHT_REVERSED = false;
            public double
                    RPM           = 435,
                    CPR           = 384.539792388;
        }

        public static class Controller {
            public double
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    kI            = 0,
                    kD            = 0,
                    kF            = 0;
        }
        public static class Position {
            public double
                    TALL = 955, // Degrees
                    MIDDLE = 330, // Degrees
                    LOWER = 0, // Degrees
                    INITIAL = 0,
                    SHARED_HIGH = 400,
                    SHARED_LOW = 100,
                    CAP_HIGH = 1210,
                    CAP_LOW = 1210,
                    CAP_PICKUP = 350,
                    MAX_POSITION = 1210,
                    MIN_POSITION = 0;
            public enum Tall {
                LOWER, MIDDLE, TALL
            }
        }
        public static class Speed {
            public double
                    NORMAL_SPEED          = 1,
                    INITIAL_SPEED         = 1,
                    SPEED_DEGREES_CHANGE          = 5;

        }
    }

    public static class LiftServoConstants {
        public static Hardware hardware = new Hardware();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String LEFT_ID            = "leftLiftServo";
            public String RIGHT_ID            = "rightLiftServo";
            public boolean LEFT_REVERSED     = false;
            public boolean RIGHT_REVERSED     = true;
        }

        public static class Position {
            public double
                    HIGH          = 209, // Degrees
                    MID           = 209, // Degrees
                    LOW           = 142, // Degrees
                    INITIAL = 10,
                    SHARED_HIGH = 0,
                    SHARED_LOW = 0,
                    CAP_HIGH = 104,
                    CAP_LOW = 76,
                    CAP_PICKUP = 54,
                    RIGHT_SERVO_OFFSET = 0;
        }
        public static class Speed {
            public double
                    SPEED_DEGREES_CHANGE          = 0.4;

        }
    }

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
            public double OPEN                       = 90; // Degrees
            public double CLOSE                      = 9; // Degrees
        }
    }

}
