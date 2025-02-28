package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.PID.PIDController;
import org.firstinspires.ftc.teamcode.util.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.util.PID.TrapezoidProfile;

public class ArmRotateIntake implements Subsystem {
    private static final double gearRatio = 10.0;
//    private static final double ticksPerRadian = ((537.7 * gearRatio) / 2) * Math.PI;

    //537.7 is the encoder resolution of the gobilda 312rpm motor
    private static final double radiansPerTick = ((6.28 / 537.7) / gearRatio);

    private static final double feedForward = -0.1;

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armRotateIntake;
    private final MotorEx armRotateIntake2;


    //TODO Not Tuned; Tune
    private final PIDController pidController = new PIDController(0.032, 0, 0);

    private final PIDController pidControllerLiftRotIncrease = new PIDController(0.14, 0, 0);

    private final ProfiledPIDController pickUpPidController = new ProfiledPIDController(0.08, 0.001, 0.002, new TrapezoidProfile.Constraints(50, 90));

    private final DigitalChannel touchSensor;


    public enum controlState {
        PLACE_ROTATE(180),
        HB_AFTER(165),
        PICK_UP_ROTATE(63),
        MANUAL_ROTATE(-10),
        MANUAL_ROTATE_REVERSE(-11),
        SWAP_STATES_ROTATE(-55),
        PRE_PICK_UP_ROTATE(75),
        HOLD_ROTATE(15),
        INCREASE_ROTATE(56);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private controlState currentState = controlState.MANUAL_ROTATE;

    private double manualPower = 0;
    private double savedPosition = startOffset;
    private double savedIncrease = startOffset;
    double startingOffset = Math.toRadians(startOffset) / radiansPerTick;

    boolean whatState = true;
    boolean sensorOnce = false;

    private boolean resetIncrease = true;

    double output;

    public ArmRotateIntake() {
        //Linking armLift in the code to the motor on the robot
        armRotateIntake = new MotorEx(hm, "armRotateIntake", Motor.GoBILDA.RPM_312);
        armRotateIntake2 = new MotorEx(hm, "armRotateIntake2", Motor.GoBILDA.RPM_312);

        armRotateIntake.setInverted(true);
        armRotateIntake2.setInverted(false);

        armRotateIntake.resetEncoder();
        armRotateIntake2.resetEncoder();

        //Setting the configuration for the motor
        armRotateIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armRotateIntake2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(.5);
        pidController.enableContinuousInput(-180, 180);
        pidController.reset();

        pidControllerLiftRotIncrease.setTolerance(.2);
        pidControllerLiftRotIncrease.enableContinuousInput(-180, 180);
        pidControllerLiftRotIncrease.reset();

        pickUpPidController.setTolerance(1);
        pickUpPidController.enableContinuousInput(-180, 180);
        pickUpPidController.reset(startingOffset);

        touchSensor = hm.get(DigitalChannel.class, "armRotateReset");
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("degrees", getRot().getDegrees());
//        packet.put("Arm rot pos", armRotateIntake.getCurrentPosition());
        packet.put("Arm rot pos2", armRotateIntake2.getCurrentPosition());
        packet.put("Saved position", savedPosition);
        dashboard.sendTelemetryPacket(packet);

        if (getTouchSensor() && !sensorOnce) {
            sensorOnce = true;
            armRotateIntake2.resetEncoder();
            savedPosition = new Rotation2d((armRotateIntake2.getCurrentPosition() + startingOffset) * radiansPerTick).getDegrees();
//            currentState = controlState.HOLD_ROTATE;
            resetIncrease = true;
        } else if (!getTouchSensor() && sensorOnce) {
            sensorOnce = false;
        }

        boolean isProfiled = false;

        switch (currentState) {
            case MANUAL_ROTATE:
//                if (getRot().getDegrees() < maxRotation || (getRot().getDegrees() > maxRotation && getRot().getDegrees() < maxRotation + 30)) {
//
//                }
                armRotateIntake.set(manualPower);
                armRotateIntake2.set(manualPower);
//                setPower(manualPower, controlState.MANUAL_ROTATE);
                return;
            case MANUAL_ROTATE_REVERSE:
                armRotateIntake.set(manualPower);
                armRotateIntake2.set(manualPower);
//                setPower(manualPower, controlState.MANUAL_ROTATE_REVERSE);
                return;
            case PICK_UP_ROTATE:
//                pickUpPidController.setSetPoint(controlState.PICK_UP_ROTATE.pos);
//                isProfiled = true;
//                pickUpPidController.setGoal(controlState.PICK_UP_ROTATE.pos);
                pidController.setSetpoint(controlState.PICK_UP_ROTATE.pos);
                break;
            case PLACE_ROTATE:
                pidController.setSetpoint(controlState.PLACE_ROTATE.pos);
                break;
            case HB_AFTER:
                pidController.setSetpoint(controlState.HB_AFTER.pos);
                break;
            case HOLD_ROTATE:
                resetIncrease = true;
                pidController.setSetpoint(savedPosition);
                break;
            case INCREASE_ROTATE:
                isProfiled = true;
                pidControllerLiftRotIncrease.setSetpoint(savedIncrease);
                break;
            case PRE_PICK_UP_ROTATE:
//                isProfiled = true;
//                pickUpPidController.setGoal(controlState.PRE_PICK_UP_ROTATE.pos);
                pidController.setSetpoint(controlState.PRE_PICK_UP_ROTATE.pos);
                break;
        }

        double currentDegrees = new Rotation2d((armRotateIntake2.getCurrentPosition() + startingOffset) * radiansPerTick).getDegrees();
//        double currentDegrees = Math.toDegrees((armRotateIntake2.getCurrentPosition() + startingOffset) * radiansPerTick);

//        if (currentDegrees < maxRotation) { // || (currentDegrees > maxRotation && currentDegrees < maxRotation + 30)) {
//            pidController.setSetpoint(maxRotation);
//        }

        output = isProfiled ? pidControllerLiftRotIncrease.calculate(currentDegrees): pidController.calculate(currentDegrees);

        if ((isProfiled && pidControllerLiftRotIncrease.atSetpoint()) || (!isProfiled && pidController.atSetpoint()) && usePIDRotationArm) {
            armRotateIntake.set(0);
            armRotateIntake2.set(0);
        } else if (usePIDRotationArm) {
            armRotateIntake.set(output);
            armRotateIntake2.set(output);
        }

        TelemetryPacket random = new TelemetryPacket();
        random.put("Rotation output", output);
        random.put("current degrees", currentDegrees);
        random.put("Rot State", currentState);
        dashboard.sendTelemetryPacket(random);
    }

    public void setPower(double power, controlState state) {

        currentState = state;

        double rotationDegrees = getRot().getDegrees();

        // && rotationDegrees > maxRotation

        if ((currentState == controlState.MANUAL_ROTATE) ) {
            manualPower = power;
        } else if (currentState == controlState.MANUAL_ROTATE_REVERSE) {
            manualPower = power;
        }
        else if (currentState == controlState.HOLD_ROTATE) {
            armRotateIntake.set(0);
            armRotateIntake2.set(0);
            savedPosition = getRot().getDegrees();
        }
        else if (currentState == controlState.PLACE_ROTATE) {
            pidController.setSetpoint(controlState.PLACE_ROTATE.pos);
        }
        else if (currentState == controlState.HB_AFTER) {
            pidController.setSetpoint(controlState.HB_AFTER.pos);
        }
        else if (currentState == controlState.PRE_PICK_UP_ROTATE) {
            pickUpPidController.setGoal(controlState.PRE_PICK_UP_ROTATE.pos);
        }
        else if (currentState == controlState.PICK_UP_ROTATE) {
            pickUpPidController.setGoal(controlState.PICK_UP_ROTATE.pos);
        }
        else {
            armRotateIntake.set(0);
            armRotateIntake2.set(0);
        }
    }

    public Rotation2d getRot() {
        double rad = (armRotateIntake2.getCurrentPosition() + startingOffset) * radiansPerTick;
        return new Rotation2d(rad);
    }

    public boolean isAtPosition(double tolerance) {
        double curPos = getRot().getDegrees();
        double desiredPos = currentState.pos;
        return Math.abs(curPos - desiredPos) <= tolerance;
    }

    public boolean getTouchSensor() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Rotate Touch", !touchSensor.getState());
        dashboard.sendTelemetryPacket(packet);
        return !touchSensor.getState();
    }

    public void increaseRot() {
        double increaseAmount = .2;
        if (getRot().getDegrees() < angleChange - increaseAmount - 1) {
            savedIncrease = resetIncrease ? getRot().getDegrees() + increaseAmount : savedIncrease + increaseAmount;
            savedIncrease = savedIncrease + increaseAmount;
            currentState = controlState.INCREASE_ROTATE;
            pidControllerLiftRotIncrease.setSetpoint(savedIncrease);

            resetIncrease = false;
        }
    }

}