package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class RoadrunnerTuning {
    //TODO: fix this (unless ftcdashbord thing doens twork)
    public static AutomaticFeedforwardTuner automaticFeedforwardTuner = new AutomaticFeedforwardTuner();
    public static BackAndForth backAndForth = new BackAndForth();
    public static DriveVelocityPIDTuner driveVelocityPIDTuner = new DriveVelocityPIDTuner();
    public static FollowerPIDTuner followerPIDTuner = new FollowerPIDTuner();
    public static LocalizationTest localizationTest = new LocalizationTest();
    public static ManualFeedforwardTuner manualFeedforwardTuner = new ManualFeedforwardTuner();
    public static MaxAngularVeloTuner maxAngularVeloTuner = new MaxAngularVeloTuner();
    public static MaxVelocityTuner maxVelocityTuner = new MaxVelocityTuner();
    public static MotorDirectionDebugger motorDirectionDebugger = new MotorDirectionDebugger();
    public static SplineTest splineTest = new SplineTest();
    public static StrafeTest strafeTest = new StrafeTest();
    public static StraightTest straightTest = new StraightTest();
    public static TrackingWheelForwardOffsetTuner trackingWheelForwardOffsetTuner = new TrackingWheelForwardOffsetTuner();
    public static TrackingWheelLateralDistanceTuner trackingWheelLateralDistanceTuner = new TrackingWheelLateralDistanceTuner();
    public static TrackWidthTuner trackWidthTuner = new TrackWidthTuner();
    public static TurnTest turnTest = new TurnTest();






    public static class AutomaticFeedforwardTuner {
        public double MAX_POWER = 0.7;
        public double DISTANCE = 100; // in
    }

    public static class BackAndForth {
        public double DISTANCE = 50; // in
    }
    public static class DriveVelocityPIDTuner {
        public double DISTANCE = 72; // in
    }

    public static class FollowerPIDTuner {
        public double DISTANCE = 48; // in
    }

    public static class LocalizationTest {
        // Nothing in here
    }
    public static class ManualFeedforwardTuner {
        public double DISTANCE = 72; // in
    }
    public static class MaxAngularVeloTuner {
        public double RUNTIME = 4.0;
    }
    public static class MaxVelocityTuner {
        public double RUNTIME = 2.0;
    }
    public static class MotorDirectionDebugger {
        public double MOTOR_POWER = 0.7;
    }
    public static class SplineTest {
        public Vector2d END_VECTOR = new Vector2d(30, 30);
    }
    public static class StrafeTest {
        public double DISTANCE = 60; // in
    }
    public static class StraightTest {
        public double DISTANCE = 60; // in
    }
    public static class TrackingWheelForwardOffsetTuner {
        public double ANGLE = 180; // deg
        public int NUM_TRIALS = 5;
        public int DELAY = 1000; // ms
    }
    public static class TrackingWheelLateralDistanceTuner {
        public int NUM_TURNS = 10;
    }
    public static class TrackWidthTuner {
        public double ANGLE = 180; // deg
        public int NUM_TRIALS = 5;
        public int DELAY = 1000; // ms

    }
    public static class TurnTest {
        public double ANGLE = 90; // deg

    }
}
