package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.Constants.centerClawPos;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.odPodLeftRetract;
import static org.firstinspires.ftc.teamcode.Constants.odPodRightRetract;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.pickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.pickUpHighClawPos;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Constants.rightClawPos90;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.HOLD_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.MANUAL_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.MANUAL_REVERSE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PICK_UP_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HOLD_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.MANUAL_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.MANUAL_ROTATE_REVERSE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Automation.AfterPlacePieceHBTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.ArmResetAfterPickUp;
import org.firstinspires.ftc.teamcode.Commands.Automation.AutoHang;
import org.firstinspires.ftc.teamcode.Commands.Automation.ClipStageOneTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.ClipStageTwoTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpClip;
import org.firstinspires.ftc.teamcode.Commands.Automation.PrePlacePieceHBTele;
import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.OdPodLift;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClawHorizontal;
import org.firstinspires.ftc.teamcode.Commands.WristClawVert;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.LiftOdPods;

@TeleOp(name = "DriveTeleop", group = "ScrapHeads")
public class DriveTeleop extends CommandOpMode {
    //Creating all the variables used in the code

    //Creating controller
    GamepadEx driver = null;

    //Creating drivetrain
    Drivetrain drivetrain = null;

    @Override
    public void initialize() {
        //Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        //Initializing the controller 1 for inputs in assignControls
        driver = new GamepadEx(gamepad1);

        // Might need to change pose2d for field centric reasons, will need to change for autos
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain.register();

        assignControls();
    }

    public void assignControls() {

        //Inputs for the drive train
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, .25));
    }
}