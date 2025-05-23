package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Subsystem {
    //Setting clawIntake variable to be set in the Claw function
    private final ServoEx clawIntake;
    private final ServoEx clawIntake2;

    private final DigitalChannel touchSensor;

    public Claw() {
        //Linking clawIntake in the code to the servo on the robot
        clawIntake = new SimpleServo(hm, "1", -1, 1);
        clawIntake2 = new SimpleServo(hm, "2", -1, 1);

        clawIntake.turnToAngle(0);
        clawIntake2.turnToAngle(0);

        touchSensor = hm.get(DigitalChannel.class, "touchClaw");
    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Touch Sensor",touchSensor.getState());
        packet.put("Claw Pos", clawIntake.getPosition());
//        dashboard.sendTelemetryPacket(packet);
    }

    public boolean getTouchSensor() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Touch State", touchSensor.getState());
        dashboard.sendTelemetryPacket(packet);
        return !touchSensor.getState();
    }

    public void setPower(double pos, double pos2) {
        //Setting the clawIntake to constantly move
        clawIntake.turnToAngle(pos);
        clawIntake2.turnToAngle(pos2);
    }
}