package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.CLIP_LIFT;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;

public class ClipStageTwoTele extends SequentialCommandGroup {
    public ClipStageTwoTele(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw, ClawWristVert wClawV, ClawRotateHorizontal rClawH) {
        addCommands(
                new liftArmIntake(lift, 1, CLIP_LIFT)
        );
    }
}
