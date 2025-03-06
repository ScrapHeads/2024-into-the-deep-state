package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftOdPods;

public class OdPodLift extends CommandBase {

    private final LiftOdPods liftOdPods;

    private final double pos;
    private final double pos2;

    public OdPodLift(LiftOdPods liftOdPods, double pos, double pos2) {
        this.liftOdPods = liftOdPods;
        this.pos = pos;
        this.pos2 = pos2;

        addRequirements(liftOdPods);
    }

    @Override
    public void initialize() {
        //Setting claw to the position in intakeClaw function
//        claw.setPower(pos, pos2);
        liftOdPods.setPower(pos, pos2);
    }

    @Override
    public void end(boolean isInterrupted) {
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("1", true);
//        dashboard.sendTelemetryPacket(packet);
        liftOdPods.setPower(0, 0);
    }

//    @Override
//    public boolean isFinished() {
//        return false;
//    }

}
