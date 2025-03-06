package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HANG_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class AutoHang extends SequentialCommandGroup {
    public AutoHang(ArmRotateIntake rotation) {
        addCommands(
                new RotateArmIntake(rotation, 1, HANG_ROTATE),
                new WaitCommand(1500),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE)
        );
    }

}
