package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Autonomous(group = "Auto", name = "Auto Test")
public class AutoTestOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();

        Action test1 = robot.drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
                ///////////////////// FIRST ELEMENT ////////////////////////
                .afterTime(0.01,AutoSeq.SampleDropPos(arm,slider))
                .strafeToConstantHeading(new Vector2d(-50,-55))
                .afterTime(0.9,AutoSeq.SampleDropYawRight(arm,slider))
                .waitSeconds(0.5)
                ///////////////////// SECOND ELEMENT ///////////////////////
                .afterTime(1.5,AutoSeq.SamplePickPos(arm,slider))
                .waitSeconds(0.7)
                .strafeToLinearHeading(new Vector2d(11,-57),0)
                .stopAndAdd(AutoSeq.SamplePickYaw(arm,slider))
                .afterTime(0.5,AutoSeq.SampleDropPos(arm,slider))
                .strafeToLinearHeading(new Vector2d(-50,-55),0)
                .stopAndAdd(AutoSeq.SampleDropYawRight(arm,slider))
                .waitSeconds(0.5)
                .afterTime(0.01,AutoSeq.HomePos(arm,slider))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-49,-37),Math.PI/2 + Math.toRadians(4))
                ////////////////////// THIRD ELEMENT ///////////////////////
                .afterTime(0.01,AutoSeq.SamplePick(arm,slider))
                .waitSeconds(1)
                .afterTime(0.01,AutoSeq.SampleDropPos(arm,slider))
                .strafeToConstantHeading(new Vector2d(-57,-49))
                .afterTime(0.9,AutoSeq.SampleDropYawLeft(arm,slider))
                .waitSeconds(1)
                .afterTime(0.01,AutoSeq.HomePos(arm,slider))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-57,-37),Math.PI/2 + Math.toRadians(4))
                //////////////////////// FOURTH ELEMENT /////////////////////
                .afterTime(0.01,AutoSeq.SamplePick(arm,slider))
                .waitSeconds(1)
                .afterTime(0.01,AutoSeq.SampleDropPos(arm,slider))
                .strafeToConstantHeading(new Vector2d(-57,-49))
                .afterTime(0.9,AutoSeq.SampleDropYawLeft(arm,slider))
                .waitSeconds(5)
                .build();

//        Action test2 = robot.drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
//                .strafeToLinearHeading(new Vector2d(-50,-55),0)
//                .afterTime(0.01,AutoSeq.HomePos(arm,slider))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-42,-30),Math.PI/2 + Math.toRadians(4))
//                .afterTime(1,AutoSeq.SamplePick(arm,slider))
//                .waitSeconds(5)
//                .build();

        if(opModeInInit()) {
            Actions.runBlocking(AutoSeq.Init(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(test1);
//            Actions.runBlocking(test2);
        }


    }
}