package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class LiftOdPods implements Subsystem {
    private final ServoEx odLiftLeft;
    private final ServoEx odLiftRight;

    public LiftOdPods() {
        odLiftLeft = new SimpleServo(hm, "odLiftLeft", -1, 1);
        odLiftRight = new SimpleServo(hm, "odLiftRight", -1, 1);

        odLiftLeft.turnToAngle(0);
        odLiftRight.turnToAngle(0);
    }

    public void periodic() {

    }

    public void setPower(double pos, double pos2) {
        odLiftLeft.turnToAngle(pos);
        odLiftRight.turnToAngle(pos2);
    }


}
