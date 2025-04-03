package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.centerClawPos;
import static org.firstinspires.ftc.teamcode.Constants.pickUpHighClawPos;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PRE_CLIP_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

public class ClipStageOneTele extends SequentialCommandGroup {
    public ClipStageOneTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV, ClawRotateHorizontal rClawH) {
        addCommands(
            new ParallelCommandGroup(
                    new RotateArmIntake(rotation, 1, PLACE_ROTATE),
                    new RotateClawHorizontal(rClawH, centerClawPos),
                    new WristClawVert(wClawV, placeClawPos)
            ),
                new liftArmIntake(lift, 1, PRE_CLIP_LIFT)
        );
    }
}
