package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.littleLowerPickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.outOfTheWay;
import static org.firstinspires.ftc.teamcode.Constants.pickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.pickUpDive;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PICK_UP_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HB_AFTER;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class ArmPlaceToPickUp extends SequentialCommandGroup {
    public ArmPlaceToPickUp(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotate rClaw) {
        addCommands(
                new RotateClaw(rClaw, outOfTheWay),
                new WaitCommand(200),
//                new RotateArmIntake(rotation, 1, HB_AFTER),//.withTimeout(200),
//                new WaitUntilCommand(() -> rotation.isAtPosition(10)),
                new liftArmIntake(lift, 1, RESET_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(4)),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
                new RotateClaw(rClaw, pickUpClawPos).withTimeout(50),
//                new WaitUntilCommand(() -> rotation.isAtPosition(3)),
                new WaitCommand(600),
                new liftArmIntake(lift, 1, PICK_UP_LIFT)
        );
    }
}
