package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw implements Subsystem {
    //Setting clawIntake variable to be set in the Claw function
    private final ServoEx clawIntake;
    private final ServoEx clawIntake2;

    private final Rev2mDistanceSensor distanceSensor;

    public Claw() {
        //Linking clawIntake in the code to the servo on the robot
        //port 1 right claw
        clawIntake = new SimpleServo(hm, "rightClaw", -1, 1);
        //port 2 left claw
        clawIntake2 = new SimpleServo(hm, "leftClaw", -1, 1);

        clawIntake.turnToAngle(0);
        clawIntake2.turnToAngle(0);

        distanceSensor = hm.get(Rev2mDistanceSensor.class, "distance");
    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Touch Sensor",touchSensor.getState());
        packet.put("Claw Pos", clawIntake.getPosition());
//        dashboard.sendTelemetryPacket(packet);

        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        TelemetryPacket sock = new TelemetryPacket();
        sock.put("Distance State", distance);
        dashboard.sendTelemetryPacket(sock);
    }

    public boolean getTouchSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance State", distance);
        dashboard.sendTelemetryPacket(packet);
        return distance <= 70;
//        return false;
    }

    public void setPower(double pos, double pos2) {
        //Setting the clawIntake to constantly move
        clawIntake.turnToAngle(pos);
        clawIntake2.turnToAngle(pos2);
    }
}