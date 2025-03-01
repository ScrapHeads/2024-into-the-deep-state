package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.intakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower2;
import static org.firstinspires.ftc.teamcode.Constants.placeClawPos;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Automation.ArmPlaceToPickUp;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto2;
import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
import org.firstinspires.ftc.teamcode.Commands.RotateClaw;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.Arrays;

@Autonomous(name = "HighBasketAuto", group = "ScrapHeads")
public class HighBasketAuto extends CommandOpMode {
    //Creating all the variables used in the code
    //Creating drivetrain
    Drivetrain drivetrain = null;

    //Creating claw
    Claw claw = null;

    //Creating rotate claw
    ClawRotate rClaw = null;

    //Creating armLiftIntake
    ArmLiftIntake armLiftIntake = null;

    //creating armRotateIntake
    ArmRotateIntake armRotateIntake = null;

    @Override
    public void initialize() {
        //Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Might need to change pose2d for field centric reasons, will need to change for autos
//        drivetrain = new Drivetrain(hardwareMap, new Pose2d(33, 63, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        drivetrain.register();

        //Initializing the claw
        claw = new Claw();
        claw.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the claw rotate
        rClaw = new ClawRotate();
        rClaw.register();

        //Initializing the armLiftIntake
        armLiftIntake = new ArmLiftIntake(armRotateIntake::getRot, armRotateIntake);
        armLiftIntake.register();

        TurnConstraints turnConstraints = new TurnConstraints(Math.PI, -Math.PI, Math.PI);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        TurnConstraints turnConstraintsSlow = new TurnConstraints(3.05, -3.05, 3.05);
        VelConstraint velConstraintSlow = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(70),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintSlow = new ProfileAccelConstraint(-30, 50);

//        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .splineToLinearHeading(new Pose2d(16, 24, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(16, 26), Math.toRadians(-47));

        TrajectoryActionBuilder pickUpPreFirstBlock = drivetrain.actionBuilder(new Pose2d(16, 26, Math.toRadians(-47)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(12, 8.5), Math.toRadians(4));

        TrajectoryActionBuilder pickUpFirstBlock = drivetrain.actionBuilder(new Pose2d(12, 8.5, Math.toRadians(4)), turnConstraints, velConstraintFast, accelConstraintFast)
                .strafeToLinearHeading(new Vector2d(19, 8.5), Math.toRadians(2));

        TrajectoryActionBuilder placeFirstBlock = drivetrain.actionBuilder(new Pose2d(19, 8.5, Math.toRadians(2)))
                .strafeToLinearHeading(new Vector2d(17.5, 25), Math.toRadians(-45));
//                .splineToLinearHeading(new Pose2d(20, 27.5, Math.toRadians(-49)), Math.toRadians(0));


        TrajectoryActionBuilder pickUpPreSecondBlock = drivetrain.actionBuilder(new Pose2d(41, -4, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(23, 28, Math.toRadians(-45)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpSecondBlock = drivetrain.actionBuilder(new Pose2d(25, 28, Math.toRadians(-45)), turnConstraints, velConstraintFast, accelConstraintFast)
                .splineToLinearHeading(new Pose2d(38, 23, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeSecondBlock = drivetrain.actionBuilder(new Pose2d(33, 23, Math.toRadians(-45)), turnConstraints, velConstraintFast, accelConstraintFast)
                .splineToLinearHeading(new Pose2d(39, -1, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder pickUpPreThirdBlock = drivetrain.actionBuilder(new Pose2d(35, -8, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(25, 15, Math.toRadians(0)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraintFast, accelConstraintFast)
                .splineToLinearHeading(new Pose2d(28, 15, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraintFast, accelConstraintFast)
                .splineToLinearHeading(new Pose2d(50.5, 0, Math.toRadians(45)), Math.toRadians(0));

        schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeStartSample.build()),
                            new PlacePieceHBAuto(armLiftIntake, armRotateIntake, claw, rClaw)
                    ),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),

                    new ParallelCommandGroup(
                            new ArmPlaceToPickUp(armLiftIntake, armRotateIntake, claw, rClaw),
                            new FollowDrivePath(drivetrain, pickUpPreFirstBlock.build())
                    ),

                    new ParallelCommandGroup(
                          new FollowDrivePath(drivetrain, pickUpFirstBlock.build()),
                          new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(1100).
                                  andThen(new RotateClaw(rClaw, placeClawPos).withTimeout(50))
                    ),

                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeFirstBlock.build()),
                            new PlacePieceHBAuto2(armLiftIntake, armRotateIntake, claw, rClaw)
                    )
                ));


    }
}
