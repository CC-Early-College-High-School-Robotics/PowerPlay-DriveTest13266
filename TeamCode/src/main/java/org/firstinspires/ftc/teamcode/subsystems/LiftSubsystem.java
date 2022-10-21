package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.LiftMotorConstants;
import static org.firstinspires.ftc.teamcode.constants.Constants.LiftServoConstants;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LiftSubsystem extends HardwareSubsystem {
    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;
    ServoEx leftLiftServo;
    ServoEx rightLiftServo;
    double motorPosition;
    double servoPosition;

    public LiftSubsystem(OpMode opMode) {
        super(opMode);
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.LEFT_ID);
        leftLiftMotor.setDirection(LiftMotorConstants.hardware.LEFT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();



        rightLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.RIGHT_ID);
        rightLiftMotor.setDirection(LiftMotorConstants.hardware.RIGHT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();

        leftLiftServo = new SimpleServo(hardwareMap, LiftServoConstants.hardware.LEFT_ID, 0, 270);
        leftLiftServo.setInverted(LiftServoConstants.hardware.LEFT_REVERSED);

        rightLiftServo = new SimpleServo(hardwareMap, LiftServoConstants.hardware.RIGHT_ID, 0, 270);
        rightLiftServo.setInverted(LiftServoConstants.hardware.RIGHT_REVERSED);
        initial();
    }


    public void high() {
        motorPosition = LiftMotorConstants.position.TALL;
        servoPosition = LiftServoConstants.position.HIGH;
        turnToPositions();
    }

    public void mid() {
        motorPosition = LiftMotorConstants.position.MIDDLE;
        servoPosition = LiftServoConstants.position.MID;
        turnToPositions();
    }

    public void low() {
        motorPosition = LiftMotorConstants.position.LOWER;
        servoPosition = LiftServoConstants.position.LOW;
        turnToPositions();
    }

    public void sharedHigh() {
        motorPosition = LiftMotorConstants.position.SHARED_HIGH;
        servoPosition = LiftServoConstants.position.SHARED_HIGH;
        turnToPositions();
    }

    public void sharedLow() {
        motorPosition = LiftMotorConstants.position.SHARED_LOW;
        servoPosition = LiftServoConstants.position.SHARED_LOW;
        turnToPositions();
    }

    public void capHigh() {
        motorPosition = LiftMotorConstants.position.CAP_HIGH;
        servoPosition = LiftServoConstants.position.CAP_HIGH;
        turnToPositions();
    }
    public void capLow() {
        motorPosition = LiftMotorConstants.position.CAP_LOW;
        servoPosition = LiftServoConstants.position.CAP_LOW;
        turnToPositions();
    }
    public void capPickUp() {
        motorPosition = LiftMotorConstants.position.CAP_PICKUP;
        servoPosition = LiftServoConstants.position.CAP_PICKUP;
        turnToPositions();
    }

    public void initial() {
        motorPosition = LiftMotorConstants.position.INITIAL;
        servoPosition = LiftServoConstants.position.INITIAL;
        turnToPositions();
    }









    public void setMotorAngle(double degrees) {
        leftLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        rightLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void increaseMotorPosition () {
        changeMotorPosition(LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseMotorPosition () {
        changeMotorPosition(-LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeMotorPosition(double degrees) {
        if (motorPosition >= LiftMotorConstants.position.MAX_POSITION) {
            if (degrees < 0) motorPosition += degrees;
            return;
        }

        if (motorPosition <= LiftMotorConstants.position.MIN_POSITION) {
            if (degrees > 0) motorPosition += degrees;
            return;
        }
        
        motorPosition += degrees;
    }



    public void increaseServoPosition () {
        changeServoPosition(LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseServoPosition () {
        changeServoPosition(-LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeServoPosition(double degrees) {
        servoPosition += degrees;
    }

    public void turnToPositions() {
        leftLiftServo.turnToAngle(servoPosition);
        rightLiftServo.turnToAngle(servoPosition + LiftServoConstants.position.RIGHT_SERVO_OFFSET);
        setMotorAngle(motorPosition);

        leftLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
    }

    @Override
    public void init() {

    }


    @Override
    public void periodic() {
        telemetry.addData("Lift Motor position", motorPosition);
        telemetry.addData("Lift Servo position", servoPosition);
    }
}
