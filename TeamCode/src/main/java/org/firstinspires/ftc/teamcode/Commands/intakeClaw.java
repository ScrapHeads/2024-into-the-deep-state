package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class intakeClaw extends CommandBase {
    //linking claw variable to the Claw subsystem
    private final Claw claw;
    //Creating the position to set the servo to
    private final double pos;
    private final double pos2;

    public intakeClaw(Claw claw, double pos, double pos2) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.claw = claw;
        this.pos = pos;
        this.pos2 = pos2;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        //Setting claw to the position in intakeClaw function
//        claw.setPower(pos, pos2);
        claw.setPower(pos, pos2);
    }

    @Override
    public void end(boolean isInterrupted) {
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("1", true);
//        dashboard.sendTelemetryPacket(packet);
        claw.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pos >= intakeClawPower && pos2 <= intakeClawPower2 && claw.getTouchSensor();
//        return true;
    }
}