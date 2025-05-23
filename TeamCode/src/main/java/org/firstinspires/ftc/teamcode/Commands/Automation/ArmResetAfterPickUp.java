package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class ArmResetAfterPickUp extends SequentialCommandGroup {
    public ArmResetAfterPickUp(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotate rClaw) {
        addCommands(
                new ParallelCommandGroup(
                        new RotateClaw(rClaw, placeClawPos),
                        new RotateArmIntake(rotation, 1, PRE_PICK_UP_ROTATE)
                ),
                new WaitUntilCommand(() -> rotation.isAtPosition(5)),
                new liftArmIntake(lift, 1, RESET_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(3))
        );
    }
}
