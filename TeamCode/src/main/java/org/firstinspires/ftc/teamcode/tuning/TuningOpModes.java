package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;


import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = Drivetrain.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(Drivetrain.class)) {
            dvf = hardwareMap -> {
                Drivetrain md = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
                LazyImu lazyImu = md.lazyImu;

                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (md.localizer instanceof Drivetrain.DriveLocalizer) {
                    Drivetrain.DriveLocalizer dl = (Drivetrain.DriveLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.leftFront, dl.leftBack, dl.rightFront, dl.rightBack)
                    ));
                    leftEncs.add(new EncoderRef(0, 0));
                    leftEncs.add(new EncoderRef(0, 1));
                    rightEncs.add(new EncoderRef(0, 2));
                    rightEncs.add(new EncoderRef(0, 3));
                } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par0, dl.par1, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    parEncs.add(new EncoderRef(0, 1));
                    perpEncs.add(new EncoderRef(0, 2));
                } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                } else {
                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
                }

                return new DriveView(
                        DriveType.MECANUM,
                        Drivetrain.PARAMS.inPerTick,
                        Drivetrain.PARAMS.maxWheelVel,
                        Drivetrain.PARAMS.minProfileAccel,
                        Drivetrain.PARAMS.maxProfileAccel,
                        encoderGroups,
                        Arrays.asList(
                                md.leftFront,
                                md.leftBack
                        ),
                        Arrays.asList(
                                md.rightFront,
                                md.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(Drivetrain.PARAMS.kS,
                                Drivetrain.PARAMS.kV / Drivetrain.PARAMS.inPerTick,
                                Drivetrain.PARAMS.kA / Drivetrain.PARAMS.inPerTick),
                        0
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}