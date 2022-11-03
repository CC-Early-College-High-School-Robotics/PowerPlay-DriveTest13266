package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.CommandSchedulerWrapper;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;

@TeleOp(name = "Main TeleOP")
public class TeleOPMain extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        BetterGamepad driver = new BetterGamepad(gamepad1);
        BetterGamepad operator = new BetterGamepad(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper();

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        LiftSubsystem lift = new LiftSubsystem(this);
        GripperSubsystem trapdoor = new GripperSubsystem(this);


        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(
                telemetry,
                drive,
                lift);

        command.addDefault(() -> telemetrySubsystem.periodic(driver, operator));

        /*
         *
         * DRIVER COMMANDS
         *
         */

        command.addDefault(() -> drive.drive(
                driver.getLeftX(), driver.getLeftY(), driver.getRightX(), DriveConstants.Drivetrain.Value.FINE_CONTROL, DriveConstants.Drivetrain.Value.FIELD_CENTRIC));

        command.add(() -> driver.get(RIGHT_BUMPER))
                .whenPressed(drive::setSlow)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(LEFT_BUMPER))
                .whenPressed(drive::setTurbo)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(A))
                .whenPressed(drive::resetImu);

        command.add(() -> driver.getTriggerPressed(LEFT_TRIGGER))
                .whenPressed(trapdoor::open);

        command.add(() -> driver.getTriggerPressed(RIGHT_TRIGGER))
                .whenPressed(trapdoor::close);

        /*
         *
         * OPERATOR COMMANDS
         *
         */

        command.add(() -> operator.get(DPAD_DOWN))
                .whenPressed(lift::initial);

        command.add(() -> operator.get(DPAD_UP))
                .whenPressed(lift::high);
        command.add(() -> operator.get(DPAD_LEFT))
                .whenPressed(lift::mid);
        command.add(() -> operator.get(DPAD_RIGHT))
                .whenPressed(lift::low);

        command.add(() -> operator.get(B))
                .whenPressed(trapdoor::open);

        command.add(() -> operator.get(A))
                .whenPressed(trapdoor::close);

        command.add(() -> operator.getLeftY() > 0.3)
                .whileHeld(lift::increaseMotorPosition);

        command.add(() -> operator.getLeftY() < -0.3)
                .whileHeld(lift::decreaseMotorPosition);

        command.add(() -> operator.getRightY() > 0.3)
                .whileHeld(lift::increaseServoPosition);

        command.add(() -> operator.getRightY() < -0.3)
                .whileHeld(lift::decreaseServoPosition);

        waitForStart();

        while (opModeIsActive()) {
            // There are two things that get run when you do this.
            // The periodic method of all defined subsystems, and
            // the runnable used on the all of the buttons.
            // This runnable will be active based on the get function of
            // the trigger which is why you have to override the get
            // method of Button to be able to use it
            CommandScheduler.getInstance().run();
        }
    }
}
