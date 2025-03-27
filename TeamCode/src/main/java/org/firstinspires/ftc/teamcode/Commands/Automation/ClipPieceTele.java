package org.firstinspires.ftc.teamcode.Commands.Automation;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;

public class ClipPieceTele extends SequentialCommandGroup {
    public ClipPieceTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV) {
        addCommands(
//                new RotateArmIntake(rotation, 1, PLACE_ROTATE),
//                new WaitUntilCommand(() -> rotation.isAtPosition(40)),
//                new liftArmIntake(lift, 1, PLACE_LIFT),
//                new WaitUntilCommand(() -> lift.isAtPosition(1)),
//                new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(500),
//                new RotateClaw(rClaw, pickUpDive).withTimeout(300),
//                new RotateArmIntake(rotation, 1, HB_AFTER),
//                new WaitUntilCommand(() -> rotation.isAtPosition(50)),
//                new liftArmIntake(lift, 1, RESET_LIFT),
//                new WaitUntilCommand(() -> lift.isAtPosition(10)),
//                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
//                new RotateClaw(rClaw, placeClawPos)
        );
    }
}
