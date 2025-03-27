package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.pickUpClawPos;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PICK_UP_LIFT_HIGH;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;

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

public class ArmPlaceToPickUp3 extends SequentialCommandGroup {
    public ArmPlaceToPickUp3(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV) {
        addCommands(
                new WristClawVert(wClawV, pickUpClawPos),
                new WaitCommand(200),
                new liftArmIntake(lift, 1, RESET_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(4)),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE),
                new WristClawVert(wClawV, placeClawPos),
                new WaitCommand(300),
//                new WaitUntilCommand(() -> rotation.isAtPosition(3)),
                new liftArmIntake(lift, 1, PICK_UP_LIFT_HIGH),
                new WristClawVert(wClawV, pickUpClawPos).withTimeout(10)
        );
    }
}
