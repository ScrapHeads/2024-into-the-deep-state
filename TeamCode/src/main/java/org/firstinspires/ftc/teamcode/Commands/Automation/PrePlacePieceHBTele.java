package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.centerClawPos;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClawHorizontal;
import org.firstinspires.ftc.teamcode.Commands.WristClawVert;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;

public class PrePlacePieceHBTele extends SequentialCommandGroup {
    public PrePlacePieceHBTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotateHorizontal wClaw, ClawWristVert vClaw) {
        addCommands(
                new RotateArmIntake(rotation, 1, PLACE_ROTATE),
                new RotateClawHorizontal(wClaw, centerClawPos),
                new WristClawVert(vClaw, placeClawPos),
                new WaitUntilCommand(() -> rotation.isAtPosition(40)),
                new liftArmIntake(lift, 1, PLACE_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(1))
        );
    }
}
