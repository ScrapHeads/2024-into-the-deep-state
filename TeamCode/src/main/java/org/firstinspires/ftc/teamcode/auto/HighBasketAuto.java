package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.centerClawPos;
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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Automation.ArmPlaceToPickUp;
import org.firstinspires.ftc.teamcode.Commands.Automation.ArmPlaceToPickUp2;
import org.firstinspires.ftc.teamcode.Commands.Automation.ArmPlaceToPickUp3;
import org.firstinspires.ftc.teamcode.Commands.Automation.ArmPlaceToPickUp4;
import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpThridBlock;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto2;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto3;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAuto4;
import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
import org.firstinspires.ftc.teamcode.Commands.RotateClawHorizontal;
import org.firstinspires.ftc.teamcode.Commands.WristClawVert;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotateHorizontal;
import org.firstinspires.ftc.teamcode.Subsystems.ClawWristVert;
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
    ClawRotateHorizontal rClaw = null;

    ClawWristVert wClawV = null;

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

        //Initializing the wrist claw
        wClawV = new ClawWristVert();
        wClawV.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the claw rotate
        rClaw = new ClawRotateHorizontal();
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
                drivetrain.kinematics.new WheelVelConstraint(40),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintSlow = new ProfileAccelConstraint(-30, 40);

        TurnConstraints turnConstraintsSupsSlow = new TurnConstraints(3.05, -3.05, 3.05);
        VelConstraint velConstraintSupsSlow = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(10),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintSupsSlow = new ProfileAccelConstraint(-5, 20);

//        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .splineToLinearHeading(new Pose2d(16, 24, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(16, 21), Math.toRadians(-47));

        TrajectoryActionBuilder pickUpPreFirstBlock = drivetrain.actionBuilder(new Pose2d(16, 21, Math.toRadians(-47)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(9, 9.5), Math.toRadians(0));

        TrajectoryActionBuilder pickUpFirstBlock = drivetrain.actionBuilder(new Pose2d(9, 9.5, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(19, 9.5), Math.toRadians(0));

        TrajectoryActionBuilder placeFirstBlock = drivetrain.actionBuilder(new Pose2d(19, 9.5, Math.toRadians(0)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(19, 22), Math.toRadians(-45));

        TrajectoryActionBuilder turnToPickUpPreSecondBlock = drivetrain.actionBuilder(new Pose2d(19, 22, Math.toRadians(-45)),  turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .turnTo(Math.toRadians(0));

        TrajectoryActionBuilder pickUpPreSecondBlock = drivetrain.actionBuilder(new Pose2d(19, 22, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(17, 21.5), Math.toRadians(-4));

        TrajectoryActionBuilder pickUpSecondBlock = drivetrain.actionBuilder(new Pose2d(17, 21.5, Math.toRadians(-4)))
                .strafeToLinearHeading(new Vector2d(20, 21.5), Math.toRadians(-4));

        TrajectoryActionBuilder placeSecondBlockTurn = drivetrain.actionBuilder(new Pose2d(20, 21, Math.toRadians(-4)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .turnTo(Math.toRadians(-45));

        TrajectoryActionBuilder placeSecondBlock = drivetrain.actionBuilder(new Pose2d(11, 17, Math.toRadians(-45)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(16, 22.5), Math.toRadians(-45));

        TrajectoryActionBuilder leaveBasketSecondBlock = drivetrain.actionBuilder(new Pose2d(16, 22.5, Math.toRadians(-45)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(24, 18), Math.toRadians(-45));

        TrajectoryActionBuilder pickUpThirdBlock = drivetrain.actionBuilder(new Pose2d(24, 18, Math.toRadians(-45)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(47, -7), Math.toRadians(90));

        TrajectoryActionBuilder placeThirdBlock = drivetrain.actionBuilder(new Pose2d(47, -7, Math.toRadians(90)), turnConstraintsSlow, velConstraintSlow, accelConstraintSlow)
                .strafeToLinearHeading(new Vector2d(21, 25), Math.toRadians(-47));

        schedule(new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeStartSample.build()),
                            new PlacePieceHBAuto(armLiftIntake, armRotateIntake, claw, wClawV),
                            new RotateClawHorizontal(rClaw, centerClawPos)
                    ),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),

                    new WaitCommand(200),

                    new ParallelCommandGroup(
                            new ArmPlaceToPickUp(armLiftIntake, armRotateIntake, claw, wClawV),
                            new FollowDrivePath(drivetrain, pickUpPreFirstBlock.build())
                    ),

                    new ParallelCommandGroup(
                          new FollowDrivePath(drivetrain, pickUpFirstBlock.build()),
                          new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(1200).
                                  andThen(new WristClawVert(wClawV, placeClawPos).withTimeout(50))
                    ),
//
                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeFirstBlock.build()),
                            new PlacePieceHBAuto2(armLiftIntake, armRotateIntake, claw, wClawV)
                    ),
//
                    new WaitCommand(200),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),
//
                new ParallelCommandGroup(
                            new ArmPlaceToPickUp2(armLiftIntake, armRotateIntake, claw, wClawV),
                            new FollowDrivePath(drivetrain, turnToPickUpPreSecondBlock.build())
                                    .andThen(new FollowDrivePath(drivetrain, pickUpPreSecondBlock.build()))
                    ),
//
                    new ParallelCommandGroup(
                        new FollowDrivePath(drivetrain, pickUpSecondBlock.build()),
                        new intakeClaw(claw, intakeClawPower, intakeClawPower2).withTimeout(1200).
                                andThen(new WristClawVert(wClawV, placeClawPos).withTimeout(50))
                    ),

                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeSecondBlockTurn.build())
                                    .andThen(
                                            new FollowDrivePath(drivetrain, placeSecondBlock.build())
                                            ),
                            new PlacePieceHBAuto3(armLiftIntake, armRotateIntake, claw, wClawV)
                    ),

                    new WaitCommand(200),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),
//
                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, leaveBasketSecondBlock.build())
                                    .andThen(
                                new FollowDrivePath(drivetrain, pickUpThirdBlock.build())
                            ),
                            new ArmPlaceToPickUp3(armLiftIntake, armRotateIntake, claw, wClawV)
                    ),

                    new PickUpThridBlock(armLiftIntake, armRotateIntake, claw, wClawV, rClaw),

                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeThirdBlock.build()),
                            new PlacePieceHBAuto4(armLiftIntake, armRotateIntake, claw, wClawV)
                    ),

                    new WaitCommand(200),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),

                    new ArmPlaceToPickUp4(armLiftIntake, armRotateIntake, claw, wClawV)
                )
        );


    }
}
