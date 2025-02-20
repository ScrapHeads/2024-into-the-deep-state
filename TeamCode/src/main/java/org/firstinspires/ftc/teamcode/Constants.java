package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Constants {
    public static HardwareMap hm;
    public static Telemetry tele;
    public static FtcDashboard dashboard;

    //Starting offset of the rotation arm in degrees was 60
    public static double startOffset = 60;

    //Using PID control for rotaion of arm and extention of arm
    public static boolean usePIDLiftArm = true;
    public static boolean usePIDRotationArm = true;

    //Rotation of the claw pos
    //Originally 0
    public static double pickUpClawPos = .38;
    //Originally 1
    public static double placeClawPos = .8;
    public static double clipClawPos = 1;

    //Claw power for intake and outtake intake wheel one or right side from back of robot
    public static double intakeClawPower = 1;
    public static double outtakeClawPower = -1;

    //Claw power for intake and outtake intake wheel two or left side from back of robot
    public static double intakeClawPower2 = -1;
    public static double outtakeClawPower2 = 1;
}