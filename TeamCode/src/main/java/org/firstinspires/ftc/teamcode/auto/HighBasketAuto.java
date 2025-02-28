package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower;
import static org.firstinspires.ftc.teamcode.Constants.outtakeClawPower2;
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
import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
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
        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(60),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-40, 60);

//        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .splineToLinearHeading(new Pose2d(16, 24, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeStartSample = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(16, 26), Math.toRadians(-47));

        TrajectoryActionBuilder pickUpPreFirstBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(15, 35.5, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder pickUpFirstBlock = drivetrain.actionBuilder(new Pose2d(16, 34.5, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(27, 28, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeFirstBlock = drivetrain.actionBuilder(new Pose2d(26, 27, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(41, -4, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder pickUpPreSecondBlock = drivetrain.actionBuilder(new Pose2d(41, -4, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(23, 28, Math.toRadians(-45)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpSecondBlock = drivetrain.actionBuilder(new Pose2d(25, 28, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(38, 23, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeSecondBlock = drivetrain.actionBuilder(new Pose2d(33, 23, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(39, -1, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder pickUpPreThirdBlock = drivetrain.actionBuilder(new Pose2d(35, -8, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(25, 15, Math.toRadians(0)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(28, 15, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(50.5, 0, Math.toRadians(45)), Math.toRadians(0));

        schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new FollowDrivePath(drivetrain, placeStartSample.build()),
                            new PlacePieceHBAuto(armLiftIntake, armRotateIntake, claw, rClaw)
                    ),

                    new intakeClaw(claw, outtakeClawPower, outtakeClawPower2).withTimeout(400),

                    new ParallelCommandGroup(
                            new ArmPlaceToPickUp(armLiftIntake, armRotateIntake, claw, rClaw)
                    )
                ));


    }
}
