package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.FollowDrivePathPP;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous(name = "TestAuto", group = "ScrapHeads")
public class TestAuto extends CommandOpMode {
    //Creating all the variables used in the code
    //Creating drivetrain
    Drivetrain drivetrain = null;

    PinpointDrive pinpointDrive = null;

    //Creating claw
    Claw claw = null;

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
//        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
//        drivetrain.register();

        pinpointDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        pinpointDrive.register();

        //Initializing the claw
        claw = new Claw();
        claw.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the armLiftIntake
        armLiftIntake = new ArmLiftIntake(armRotateIntake::getRot, armRotateIntake);
        armLiftIntake.register();

        //Initializing the armLiftClipper
//        armLiftClipper = new ArmLiftClipper();
//        armLiftClipper.register();

        //Initializing the armRotateClipper
//        armRotateClipper = new ArmRotateClipper();
//        armRotateClipper.register();

        TrajectoryActionBuilder testTrajectory = drivetrain.actionBuilder(new Pose2d(0, 0, 0))
                .splineToLinearHeading(new Pose2d(10, 0, 0), 0);

        TrajectoryActionBuilder nextTrajectory = drivetrain.actionBuilder(new Pose2d(6, -34, 90))
                .splineToLinearHeading(new Pose2d(-38, -40, 90), 0)
                .splineToLinearHeading(new Pose2d(-40, -10, 90), 0)
                .splineToLinearHeading(new Pose2d(-50, -53, 90), 0);

        schedule(new SequentialCommandGroup(

                new FollowDrivePathPP(pinpointDrive, testTrajectory.build())

//                new FollowDrivePath(drivetrain, testTrajectory.build()),
//                new FollowDrivePath(drivetrain, nextTrajectory.build()),

//        new ParallelCommandGroup(
////                new intakeClaw(claw, 1)
//
//                ),
//        new ParallelCommandGroup(
//                new FollowDrivePath(drivetrain, nextTrajectory.build())
////                new intakeClaw(claw, 1)
//                )
////                new WaitCommand(500),
////                new intakeClaw(claw, 0)
        ));


    }
}
