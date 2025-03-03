package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.littleLowerPickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.outOfTheWay;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PICK_UP_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class ArmPlaceToPickUp3 extends SequentialCommandGroup {
    public ArmPlaceToPickUp3(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotate rClaw) {
        addCommands(
                new RotateClaw(rClaw, outOfTheWay).withTimeout(2000),
                new liftArmIntake(lift, 1, RESET_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(6)),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
                new RotateClaw(rClaw, littleLowerPickUpClawPos).withTimeout(50),
                new WaitCommand(1000),
//                new WaitUntilCommand(() -> rotation.isAtPosition(3)),
                new liftArmIntake(lift, 1, PICK_UP_LIFT)
        );
    }
}
