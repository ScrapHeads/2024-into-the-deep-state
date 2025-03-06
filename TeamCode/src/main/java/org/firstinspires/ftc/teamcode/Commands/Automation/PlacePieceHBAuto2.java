package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HANG_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class PlacePieceHBAuto2 extends SequentialCommandGroup {
    public PlacePieceHBAuto2(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawRotate rClaw) {
        addCommands(
                new RotateArmIntake(rotation, 1, HANG_ROTATE),
                new WaitCommand(1300),
                new WaitUntilCommand(() -> rotation.isAtPosition(2)),
                new liftArmIntake(lift, 1, PLACE_LIFT),
                new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(200),
                new WaitUntilCommand(() -> lift.isAtPosition(10)),
                new RotateClaw(rClaw, placeClawPos)
        );
    }
}
