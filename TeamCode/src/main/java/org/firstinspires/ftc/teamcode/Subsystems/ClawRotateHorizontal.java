package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawRotateHorizontal implements Subsystem {
    //Setting clawIntake variable to be set in the Claw function
    private final ServoEx clawRotate;

    public ClawRotateHorizontal() {
        //Linking clawIntake in the code to the servo on the robot
        //port 0
        clawRotate = new SimpleServo(hm, "rotate", -100, 100, AngleUnit.DEGREES);

//        clawRotate.turnToAngle(0);
    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Claw Pos", clawRotate.getPosition());
        dashboard.sendTelemetryPacket(packet);
    }

    public void setPower(double pos) {
        //Setting the clawIntake to constantly move
//        clawRotate.turnToAngle(pos);
        clawRotate.setPosition(pos);
    }
}
