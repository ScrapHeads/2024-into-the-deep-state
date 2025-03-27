package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

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

public class PlacePieceHBAuto extends SequentialCommandGroup {
    public PlacePieceHBAuto(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV) {
        addCommands(
                new RotateArmIntake(rotation, 1, PLACE_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(15)),
                new liftArmIntake(lift, 1, PLACE_LIFT),
                new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(200),
                new WaitUntilCommand(() -> lift.isAtPosition(10)),
                new WristClawVert(wClawV, placeClawPos)
//                new RotateClaw(rClaw, pickUpDive).withTimeout(300),
//                new RotateArmIntake(rotation, 1, HB_AFTER),
//                new WaitUntilCommand(() -> rotation.isAtPosition(50)),
//                new liftArmIntake(lift, 1, RESET_LIFT),
//                new WaitUntilCommand(() -> lift.isAtPosition(8)),
//                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
//                new RotateClaw(rClaw, placeClawPos)
        );
    }
}
