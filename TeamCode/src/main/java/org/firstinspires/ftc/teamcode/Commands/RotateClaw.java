package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;

public class RotateClaw extends CommandBase {
    //linking claw variable to the Claw subsystem
    private final ClawRotate claw;
    //Creating the position to set the servo to
    private final double pos;

    public RotateClaw(ClawRotate claw, double pos) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.claw = claw;
        this.pos = pos;

        addRequirements();
    }

    @Override
    public void initialize() {
        //Setting claw to the position in intakeClaw function
        claw.setPower(pos);
    }

    @Override
    public void end(boolean isInterrupted) {
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("1", true);
//        dashboard.sendTelemetryPacket(packet);
        claw.setPower(0);
    }

    @Override
    public boolean isFinished() {
//        return pos <= 0 && claw.getTouchSensor();
        return true;
    }
}

