package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadExEx extends GamepadEx {
    public GamepadExEx(Gamepad gamepad) {
        super(gamepad);
    }
//
//    @Override
//    public double getLeftX() {
//        return gamepad.left_stick_x >= value.STICK_THRESHOLD ? gamepad.left_stick_x : 0;
//    }
//
//    @Override
//    public double getLeftY() {
//        return -gamepad.left_stick_y >= value.STICK_THRESHOLD ? gamepad.left_stick_y : 0;
//    }
//
//    @Override
//    public double getRightX() {
//        return gamepad.right_stick_x >= value.STICK_THRESHOLD ? gamepad.right_stick_x : 0;
//    }
//
//    @Override
//    public double getRightY() {
//        return -gamepad.right_stick_y >= value.STICK_THRESHOLD ? gamepad.right_stick_y : 0;
//    }

    public boolean getTriggerPressed(GamepadKeys.Trigger trigger) {
        return getTrigger(trigger) > 0.3;
    }

    public boolean getLeftTouchingEdge() {
        return (Math.sqrt((Math.pow(getLeftX(), 2)) + (Math.pow(getLeftY(), 2)))) >  0.3;
    }

    public boolean getRightTouchingEdge() {
        return (Math.sqrt((Math.pow(getRightX(), 2)) + (Math.pow(getRightY(), 2)))) >  0.3;
    }

    public boolean get(GamepadKeys.Button button) {
        return getButton(button);
    }

    public boolean get(GamepadKeys.Trigger trigger) {
        return getTrigger(trigger) >  0.3;
    }



    /**
     * For some stupid reason, the left stick is set to negative, but the right ones
     * y value is not set to negatiive. this is very dumb so i fixed it
     * I dont really see how this was intentional so i dont think it was
     */
    public double getRightY() {
        return -gamepad.right_stick_y;
    }
    
    public double getLeftStickToDegrees() {
        return Math.toDegrees(Math.atan2(getLeftX(), -getLeftY()));
    }

    public double getRightStickToDegrees() {
//        double value = Math.toDegrees(Math.atan2(getRightX(), getRightY())) + 180;
//        boolean negative = Math.toDegrees(Math.atan2(getRightX(), getRightY())) < 0;
//        return negative ? value - 360:
//                value;
        return Math.toDegrees(Math.atan2(getRightX(), -getRightY()));
    }


}






