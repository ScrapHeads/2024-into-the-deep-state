package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;

import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.teleops.MainTeleop;

public class ArmResetAfterPickUp extends SequentialCommandGroup {
    public ArmResetAfterPickUp(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV) {
        addCommands(
            new ParallelCommandGroup(
                    new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
                    new liftArmIntake(lift, 1, RESET_LIFT)
//                    new WristClawVert(wClawV, placeClawPos)
                )
//                new WaitUntilCommand(() -> rotation.isAtPosition(10)),
//                new WaitUntilCommand(() -> lift.isAtPosition(3))
        );
    }
}
