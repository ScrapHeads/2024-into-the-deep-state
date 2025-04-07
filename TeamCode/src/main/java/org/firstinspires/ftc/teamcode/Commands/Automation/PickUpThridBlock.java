package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.centerClawPos;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.pickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Constants.rightClawPos90;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PICK_UP_THIRD_BLOCK_AUTO_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_THIRD_BLOCK_AUTO;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

public class PickUpThridBlock extends SequentialCommandGroup {
    public PickUpThridBlock(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV, ClawRotateHorizontal rClawH) {
        addCommands(
                new ParallelCommandGroup(
                        new RotateArmIntake(rotation, 1, ArmRotateIntake.controlState.PRE_PICK_UP_THIRD_BLOCK_AUTO),
                        new WristClawVert(wClawV, pickUpClawPos),
                        new RotateClawHorizontal(rClawH, rightClawPos90)
                        ),
                new WaitCommand(500),

                new liftArmIntake(lift, 1, PICK_UP_THIRD_BLOCK_AUTO_LIFT),

                new ParallelCommandGroup(
                        new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(700),
                        new WaitCommand(100).andThen(
                                new RotateArmIntake(rotation, 1, PICK_UP_THIRD_BLOCK_AUTO)
                        )
                ),

                new ParallelCommandGroup(
                        new liftArmIntake(lift, 1, RESET_LIFT),
                        new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
                        new WristClawVert(wClawV, placeClawPos),
                        new RotateClawHorizontal(rClawH, centerClawPos)

                )
        );
    }
}
