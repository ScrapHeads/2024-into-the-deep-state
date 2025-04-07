package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.CLIP_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;

public class ClipStageTwoTele extends SequentialCommandGroup {
    public ClipStageTwoTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV, ClawRotateHorizontal rClawH) {
        addCommands(
                new ParallelCommandGroup(
                        new liftArmIntake(lift, 1, CLIP_LIFT),
                        new RotateArmIntake(rotation, 1, ArmRotateIntake.controlState.PLACE_ROTATE),
                        new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(300)
                ),

                new WaitCommand(800),

                new ParallelCommandGroup(
                        new RotateArmIntake(rotation, 1, ArmRotateIntake.controlState.PICK_UP_ROTATE),
                        new liftArmIntake(lift,1, RESET_LIFT)
                )
        );
    }
}
