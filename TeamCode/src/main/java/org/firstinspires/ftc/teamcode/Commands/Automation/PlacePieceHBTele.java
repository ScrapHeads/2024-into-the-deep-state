package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower2;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HB_AFTER;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class PlacePieceHBTele extends SequentialCommandGroup {
    public PlacePieceHBTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotate rClaw) {
        addCommands(
                new RotateArmIntake(rotation, 1, PLACE_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(40)),
                new liftArmIntake(lift, 1, PLACE_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(0.5)),
                new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(500)
//                new RotateArmIntake(rotation, 1, HB_AFTER),
//                new WaitUntilCommand(() -> rotation.isAtPosition(15)),
//                new liftArmIntake(lift, 1, RESET_LIFT),
//                new WaitUntilCommand(() -> lift.isAtPosition(10)),
//                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE)

        );
    }
}
