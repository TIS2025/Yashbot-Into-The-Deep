package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;
import org.opencv.core.Mat;

@Autonomous(group = "Auto", name = "Auto Specimen")
public class Auto_Specimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(6,-64),-Math.PI/2));
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();


        Action autoSequence = drive.actionBuilder(new Pose2d(new Vector2d(6,-64),-Math.PI/2))
                //FIRST SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenDropPos(arm,slider))
                .strafeToConstantHeading(new Vector2d(4,-32))
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //SAMPLE 1 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromSpecimen(arm,slider))
                .splineToLinearHeading(new Pose2d(new Vector2d(28,-32),Math.PI/12),Math.PI/4)
                .stopAndAdd(AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(new Vector2d(32,-44), -Math.PI/4)
                .stopAndAdd(AutoSeq.SampleDropObsZone(arm,slider))
                //SAMPLE 2 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .strafeToLinearHeading(new Vector2d(38,-32),Math.PI/12)
                .stopAndAdd(AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(new Vector2d(38,-44), -Math.PI/4)
                .stopAndAdd(AutoSeq.SampleDropObsZone(arm,slider))
                //SAMPLE 3 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .strafeToLinearHeading(new Vector2d(48,-32),Math.PI/12)
                .stopAndAdd(AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(new Vector2d(42,-44), -Math.PI/4)
                .stopAndAdd(AutoSeq.SampleDropObsZone(arm,slider))
                //FIRST SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPosFromSampleDrop(arm,slider))
                .build();

        if(opModeInInit()) {
            Actions.runBlocking(AutoSeq.SpecimenInit(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(autoSequence);
        }
    }
}
