package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

public class PickUpClip extends SequentialCommandGroup {
    public PickUpClip(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV, ClawRotateHorizontal rClawH) {
        addCommands(
                new ParallelCommandGroup(
                        new RotateArmIntake(rotation, 1, ArmRotateIntake.controlState.PICK_UP_ROTATE_FLOOR),
                        new WristClawVert(wClawV, pickUpHighClawPos),
                        new RotateClawHorizontal(rClawH, centerClawPos)
                        ),
                new WaitUntilCommand(() -> rotation.isAtPosition(5)),
                new liftArmIntake(lift, 1, ArmLiftIntake.controlState.PICK_UP_LIFT_FLOOR_LOW)
        );
    }
}
