package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "PracticeTeleop", group = "ScrapHeads")
public class PracticeTeleop extends CommandOpMode {
    //Creating all the variables used in the code

    //Creating controller
    GamepadEx driver = null;
    GamepadEx driver2 = null;

    //Creating drivetrain
    Drivetrain drivetrain = null;

    //Creating claw
    Claw claw = null;

    //Creating armLiftIntake
    ArmLiftIntake armLiftIntake = null;

    //creating armRotateIntake
    ArmRotateIntake armRotateIntake = null;

    public enum PickUpStates {
        STATE_ONE,
        STATE_TWO,
        STATE_THREE
    }

    public enum ClipperStates {
        STATE_ONE,
        STATE_TWO
    }

    public enum controllerStates {
        CONTROLLER_ONE,
        CONTROLLER_TWO
    }

    private PickUpStates currentPickUpState = PickUpStates.STATE_THREE;

    private ClipperStates currentClipperStates = ClipperStates.STATE_TWO;

    private controllerStates currentController = controllerStates.CONTROLLER_TWO;

    private boolean isSlowMode = false;

    @Override
    public void initialize() {
        //Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        //Initializing the controller 1 for inputs in assignControls
        driver = new GamepadEx(gamepad1);

        //Initializing the controller 2 for inputs in assignControls
        driver2 = new GamepadEx(gamepad2);

        // Might need to change pose2d for field centric reasons, will need to change for autos
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain.register();

//        //Initializing the claw intake
//        claw = new Claw();
//        claw.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the armLiftIntake
        armLiftIntake = new ArmLiftIntake(armRotateIntake::getRot);
        armLiftIntake.register();

        assignControls();
    }

    public void assignControls() {
        //Inputs for the drive train
//        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver2, 1));
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

//        driver.getGamepadButton(START)
//                .whenPressed(new InstantCommand(() -> currentController = controllerStates.CONTROLLER_ONE));
//
//        driver2.getGamepadButton(START)
//                .whenPressed(new InstantCommand(() -> currentController = controllerStates.CONTROLLER_TWO));
//
//        new Trigger(() -> currentController == controllerStates.CONTROLLER_ONE)
//                .whenActive(
//                        new DriveContinous(drivetrain, driver, 1)
//                );
//
//        new Trigger(() -> currentController == controllerStates.CONTROLLER_TWO)
//                .whenActive(
//                        new DriveContinous(drivetrain, driver2, 1)
//                );


        new Trigger(() -> isSlowMode)
                .whileActiveOnce(new DriveContinous(drivetrain, driver, 0.3));

        new Trigger(() -> !isSlowMode)
                .whileActiveOnce(new DriveContinous(drivetrain, driver, 1.0));

        //Statements for in game functions controller one

        //Inputs for the armLiftIntake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whenActive(new liftArmIntake(armLiftIntake, .25, MANUAL_REVERSE))
                .whenInactive(new liftArmIntake(armLiftIntake, 0, HOLD_LIFT));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(new liftArmIntake(armLiftIntake, -.5, MANUAL_LIFT))
                .whenInactive(new liftArmIntake(armLiftIntake, 0, HOLD_LIFT));

        //Inputs for the armRotateIntake
        driver.getGamepadButton(DPAD_LEFT)
                .whenPressed(new RotateArmIntake(armRotateIntake, 0.75, MANUAL_ROTATE))
                .whenReleased(new RotateArmIntake(armRotateIntake, 0, HOLD_ROTATE));
        driver.getGamepadButton(DPAD_RIGHT)
                .whenPressed(new RotateArmIntake(armRotateIntake, -0.5, MANUAL_ROTATE))
                .whenReleased(new RotateArmIntake(armRotateIntake, 0, HOLD_ROTATE));

        //Pid controls

        driver.getGamepadButton(Y)
                .whenPressed(new liftArmIntake(armLiftIntake, -.5, PLACE_LIFT));
//        driver.getGamepadButton(Y)
//                .whenPressed(
//                        new ParallelCommandGroup(
//                                new PlacePieceHBTele(armLiftIntake, armRotateIntake, claw),
//                                new InstantCommand(() -> {isSlowMode = true;})
//                        ).whenFinished(() -> {isSlowMode = false;})
//                );
//
//        driver.getGamepadButton(X)
//                .whenPressed(new InstantCommand(this::advancedPickUpStates));
//
//        new Trigger(() -> currentPickUpState == PickUpStates.STATE_ONE)
//                .whenActive(
//                        new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
//                )
//                .whileActiveOnce(new InstantCommand(() -> {isSlowMode = true;}));
//
//        new Trigger(() -> currentPickUpState == PickUpStates.STATE_TWO)
//                .whenActive(
//                        new SequentialCommandGroup(
//                                new RotateArmIntake(armRotateIntake, 1, PICK_UP_ROTATE),
//                                new intakeClaw(claw, -1).andThen(
//                                        new liftArmIntake(armLiftIntake, 1, RESET_LIFT),
//                                        new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
//                                )
//                        )
//                );
//
//        new Trigger(() -> currentPickUpState == PickUpStates.STATE_THREE)
//                .whileActiveOnce(new InstantCommand(() -> {isSlowMode = false;}))
//                .whenActive(
//                        new ParallelCommandGroup(
//                                new RotateArmIntake(armRotateIntake, 1, TUCK_ROTATE),
//                                new intakeClaw(claw, 0),
//                                new DriveContinous(drivetrain, driver, 1)
//                        )
//                );
//
//        driver.getGamepadButton(BACK)
//                .whenPressed(new HangEndGame(armLiftIntake, armRotateIntake, climber));

        //Inputs for the claw intake
//        driver.getGamepadButton(B)
//                .whenPressed(new intakeClaw(claw, 1))
//                .whenReleased(new intakeClaw(claw, 0));
//        driver.getGamepadButton(A)
//                .whenPressed(new intakeClaw(claw, -1))
//                .whenReleased(new intakeClaw(claw, 0));

        //Trigger example don't uncomment
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, 1))
//                .whenInactive(new intakeClaw(claw, 0));
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, -1))
//                .whenInactive(new intakeClaw(claw, 0));
    }

    private void advancedPickUpStates() {
        switch(currentPickUpState) {
            case STATE_ONE:
                currentPickUpState = PickUpStates.STATE_TWO;
                break;
            case STATE_TWO:
                currentPickUpState = PickUpStates.STATE_THREE;
                break;
            case STATE_THREE:
                currentPickUpState = PickUpStates.STATE_ONE;
                break;
        };
    }

    private void advancedClipperStates() {
        switch(currentClipperStates) {
            case STATE_ONE:
                currentClipperStates = ClipperStates.STATE_TWO;
                break;
            case STATE_TWO:
                currentClipperStates = ClipperStates.STATE_ONE;
                break;
        };
    }

    private void whatController() {
        switch(currentController) {
            case CONTROLLER_ONE:
                currentController = controllerStates.CONTROLLER_TWO;
                break;
            case CONTROLLER_TWO:
                currentController = controllerStates.CONTROLLER_ONE;
                break;
        };
    }
}
