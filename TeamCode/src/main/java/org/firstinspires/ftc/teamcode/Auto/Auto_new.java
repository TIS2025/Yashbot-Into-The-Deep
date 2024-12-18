package org.firstinspires.ftc.teamcode.Auto;

import android.net.wifi.aware.WifiAwareNetworkInfo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Sequences.FinalAutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Config
@Autonomous(group = "Auto", name = "Auto new")
public class Auto_new extends LinearOpMode {

    MecanumDrive drive;

    public static double afterTime1 = 0.4;
    public static double wait1 = 1.2;
    public static double afterTime2_1 = 0.3;
    public static double afterTime2_2 = 0.01;
    public static double wait2_1 = 3;
    public static double wait2_2 = 3;
    public static double wait2_3 = 3;
    public static double afterTime3_1 = 3;
    public static double wait3_1 = 3;
    public static double wait3_2 = 3;
    public static double wait3_3 = 3;
    public static double afterTime4_1 = 3;
    public static double wait4_1 = 3;
    public static double wait4_2 = 3;
    public static double wait4_3 = 3;
    public static double afterTime5_1 = 3;
    public static double wait5_1 = 3;
    public static double wait5_2 = 3;
    public static double wait5_3 = 3;

    public static Vector2d sample1drop = new Vector2d(-50,-55);
    public static Vector2d sample2pick = new Vector2d(11,-57);
    public static Vector2d sample2drop = new Vector2d(-50,-55);
    public static Vector2d sample3pick = new Vector2d(-48,-37);
    public static Vector2d sample3drop = new Vector2d(-59,-48.5);
    public static Vector2d sample4pick = new Vector2d(-57,-37);
    public static Vector2d sample4drop = new Vector2d(-59,-48);
    public static Vector2d sample5pick = new Vector2d(-63,-35);
    public static Vector2d sample5drop = new Vector2d(-59,-48);
    public static Vector2d park = new Vector2d(-24,-13);



    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(-40,-60),0));
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();

        Action autoSequence = drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
                ///////////////////// FIRST SAMPLE ////////////////////////
//                .afterTime(0.01, FinalAutoSeq.SampleDropPosYawRight(arm,slider))
                .strafeToConstantHeading(sample1drop)
                .waitSeconds(0.3)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                ///////////////////// SECOND SAMPLE ///////////////////////
//                .afterTime(0.3,FinalAutoSeq.SamplePickPosFullExtRightYaw(arm,slider))
//                .strafeToLinearHeading(sample2pick,0)
//                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
//                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawRight(arm,slider))
//                .strafeToLinearHeading(sample2drop,0)
//                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(1)
//                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider))
                .waitSeconds(wait2_1)
                .strafeToLinearHeading(sample3pick,Math.PI/2 + Math.toRadians(4))
                ////////////////////// THIRD SAMPLE ///////////////////////
                .afterTime(0.01,FinalAutoSeq.SamplePick(arm))
                .waitSeconds(wait3_1)
//                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider))
                .strafeToConstantHeading(sample3drop)
                .afterTime(afterTime3_1,FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(wait3_2)
//                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider))
                .waitSeconds(wait3_3)
                .strafeToLinearHeading(sample4pick,Math.PI/2 + Math.toRadians(4))
                //////////////////////// FOURTH ELEMENT /////////////////////
                .afterTime(0.01,FinalAutoSeq.SamplePick(arm))
                .waitSeconds(wait4_1)
//                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider))
                .strafeToConstantHeading(sample4drop)
                .afterTime(afterTime4_1,FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(wait4_2)
//                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider))
                .waitSeconds(wait4_3)
                .strafeToLinearHeading(sample5pick,Math.PI/2 + Math.toRadians(4))
                //////////////////////// FIFTH ELEMENT //////////////////////
                .afterTime(0.01,FinalAutoSeq.SamplePick(arm))
                .waitSeconds(wait5_1)
//                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider))
                .strafeToConstantHeading(sample5drop)
                .afterTime(afterTime5_1,FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(wait5_2)
                .afterTime(0.01,AutoSeq.SampleInit(arm,slider))
                .strafeToConstantHeading(park)
                .waitSeconds(wait5_3)
                .build();

        Action testSeq = drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extInit,MotorConst.turretInit))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosFullExtRightYaw(arm,slider,MotorConst.extHighBucketDrop,MotorConst.turretUp))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extHorizontalMax,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extHighBucketDrop))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider,MotorConst.extHighBucketDrop))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extHorizontalMax,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.TeleOpInit(arm,slider,MotorConst.extHighBucketDrop))
                .build();

        if(opModeInInit()) {
            Actions.runBlocking(FinalAutoSeq.Init(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(testSeq);
        }


    }
}
