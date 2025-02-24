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

import java.util.function.Supplier;

public class ArmLiftIntake implements Subsystem {
    private static final double ticksToInches = 31;

    //TODO Not Tuned; Tune
    private final PIDController pidController = new PIDController(0.35, 0, 0);

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftIntake;
    private final MotorEx armLiftIntake2;

    private Supplier<Rotation2d> rotSupplier;

    private ArmRotateIntake armRotateIntake = null;

    private final DigitalChannel magneticSensor;

    public enum controlState {
        PLACE_LIFT(22),
        PICK_UP_LIFT(4.1),
        RESET_LIFT(0),
        MANUAL_LIFT(-2),
        MANUAL_REVERSE(-3),
        SWAP_STATES_LIFT(-60),
        HOLD_LIFT(.1);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private ArmLiftIntake.controlState currentState = controlState.HOLD_LIFT;

    private double manualPower = 0;
    private double savedPosition = 0;

    boolean whatState = true;
    boolean sensorOnce = false;

    private boolean increaseRotation = false;


    public ArmLiftIntake(Supplier<Rotation2d> rotSupplier, ArmRotateIntake armRotateIntake) {
        //Linking armLift in the code to the motor on the robot
        armLiftIntake = new MotorEx(hm, "armLiftIntake", Motor.GoBILDA.RPM_1150);
        armLiftIntake2 = new MotorEx(hm, "armLiftIntake2", Motor.GoBILDA.RPM_1150);

        this.rotSupplier = rotSupplier;
        this.armRotateIntake = armRotateIntake;

        armLiftIntake.setInverted(false);
        armLiftIntake2.setInverted(true);

        armLiftIntake.resetEncoder();
        armLiftIntake2.resetEncoder();

        //Setting the configuration for the motor
        armLiftIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armLiftIntake2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.enableContinuousInput(-100, 180);

        pidController.setTolerance(.15);

        magneticSensor = hm.get(DigitalChannel.class, "magnet");
    }

    @Override
    public void periodic() {
        //setPower(armLiftIntake.get());
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("ElevatorTicks", armLiftIntake.getCurrentPosition());
        packet.put("ElevatorTicks2", armLiftIntake2.getCurrentPosition());

        packet.put("Cos of Rot", rotSupplier.get().getCos());
//        dashboard.sendTelemetryPacket(packet);

        double maxExtensionIn = getMaxExtensionIn();

        if (getTouchSensor() && !sensorOnce) {
            sensorOnce = true;
            armLiftIntake2.resetEncoder();
            savedPosition = getCurrentExtensionIn();
        } else if (!getTouchSensor() && sensorOnce) {
            sensorOnce = false;
        }

        switch (currentState) {
            case MANUAL_REVERSE:
                setPower(manualPower, controlState.MANUAL_REVERSE);
//                armLiftIntake.set(manualPower);
//                armLiftIntake2.set(manualPower);
                return;
            case MANUAL_LIFT:
                if (increaseRotation) {
                    armRotateIntake.increaseRot();
                    pidController.setSetpoint(maxExtensionIn);
                    break;
                }
                pidController.setSetpoint(maxExtensionIn);
//                setPower(manualPower, controlState.MANUAL_LIFT);
//                armLiftIntake.set(manualPower);
//                armLiftIntake2.set(manualPower);
                break;
            case HOLD_LIFT:
                if (savedPosition > maxExtensionIn) {
                    savedPosition = maxExtensionIn;
                }
                pidController.setSetpoint(savedPosition);
                break;
            case PLACE_LIFT:
                pidController.setSetpoint(controlState.PLACE_LIFT.pos);
                break;
            case PICK_UP_LIFT:
                pidController.setSetpoint(controlState.PICK_UP_LIFT.pos);
                break;
            case RESET_LIFT:
                pidController.setSetpoint(controlState.RESET_LIFT.pos);
                break;
        }

        double currentExtension = getCurrentExtensionIn();

        if (currentExtension >= maxExtensionIn || pidController.getSetpoint() > maxExtensionIn) {
            pidController.setSetpoint(maxExtensionIn);
        }

        double output = -pidController.calculate(currentExtension);

        if (pidController.atSetpoint() && usePIDLiftArm) {
            armLiftIntake.set(0);
            armLiftIntake2.set(0);
        } else if (usePIDLiftArm) {
            armLiftIntake.set(output);
            armLiftIntake2.set(output);
        }

        TelemetryPacket random = new TelemetryPacket();
        random.put("lift output", output);
//        packet.put("Set position", pidController.getSetPoint());
//        packet.put("Max Extension", maxExtensionIn);
        random.put("Current Extension", currentExtension);
//        random.put("Current State", currentState);
        dashboard.sendTelemetryPacket(random);
    }

    public void checkState() {
        if (whatState) {
            whatState = false;
            setPower(1, controlState.PLACE_LIFT);
        } else {
            whatState = true;
            setPower(1, controlState.RESET_LIFT);
        }
    }

    public void setPower(double power, controlState state) {
        //Setting the lift to the power in MainTeleop
//        if (state == controlState.SWAP_STATES_LIFT) {
//            checkState();
//        }

        double currentExtension = getCurrentExtensionIn();
        double maxExtensionIn = getMaxExtensionIn();
        double currentRotation = rotSupplier.get().getDegrees();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Max Extension", maxExtensionIn);
        packet.put("Current Extension", currentExtension);
//        packet.put("Tick lift", armLiftIntake2.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        currentState = state;

        //To increase the lifts rotation if at max extension below where the max extension equations change
        if (currentExtension >= (maxExtensionIn - .25) && power < 0 && currentState == controlState.MANUAL_LIFT && currentRotation <= angleChange) {
            pidController.setSetpoint(maxExtensionIn);
            increaseRotation = true;
        }
        else if (currentExtension < maxExtensionIn && power < 0 && currentState == controlState.MANUAL_LIFT) {
            pidController.setSetpoint(maxExtensionIn);
            increaseRotation = false;
//            armLiftIntake.set(power);
//            armLiftIntake2.set(power);
//            manualPower = power;
        } else if (power > 0 && currentState == controlState.MANUAL_REVERSE) {
            increaseRotation = false;
            armLiftIntake.set(power);
            armLiftIntake2.set(power);
            manualPower = power;
        }
        else if (currentState == controlState.PLACE_LIFT) {
            pidController.setSetpoint(controlState.PLACE_LIFT.pos);
        }
        else if (currentState == controlState.PICK_UP_LIFT) {
            pidController.setSetpoint(controlState.PICK_UP_LIFT.pos);
        }
        else if (currentState == controlState.RESET_LIFT) {
            pidController.setSetpoint(controlState.RESET_LIFT.pos);
        }
        else { // power is 0
            armLiftIntake.set(0);
            armLiftIntake2.set(0);
            savedPosition = currentExtension;
            currentState = controlState.HOLD_LIFT;
        }
    }

    private double getMaxExtensionIn() {
        double maxExt = 0;
        double capExt = 21;

        double rotation = rotSupplier.get().getDegrees();

        if (rotation < angleChange && rotation > 30) {
            maxExt = ((12.441 / Math.abs (rotSupplier.get().getCos()) ) * .93934) - 18.7;
        } else if (rotation >= angleChange || rotation <= -170) {
            maxExt = (29.712 / Math.abs (rotSupplier.get().getSin())) - 18.7;
        }
        if (maxExt > capExt) {
            maxExt = capExt;
        }

        return maxExt;
    }

    private double getCurrentExtensionIn() {
        double currentExt = Math.abs(armLiftIntake2.getCurrentPosition() / ticksToInches);
        return currentExt;
    }

    public boolean isAtPosition(double tolerance) {
        double curPos = Math.abs(armLiftIntake2.getCurrentPosition() / ticksToInches);
        double desiredPos = currentState.pos;
        return Math.abs(curPos - desiredPos) <= tolerance;
    }

    public boolean getTouchSensor() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Lift Touch", !magneticSensor.getState());
        dashboard.sendTelemetryPacket(packet);

        return !magneticSensor.getState();
    }

}