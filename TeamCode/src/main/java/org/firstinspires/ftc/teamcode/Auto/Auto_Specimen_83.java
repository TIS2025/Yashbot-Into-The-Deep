package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.Arrays;

@Autonomous(group = "Auto", name = "Auto Specimen 1 + 3")
public class Auto_Specimen_83 extends LinearOpMode {

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

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
                //PRELOAD SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenDropPos(arm,slider))
                .strafeToConstantHeading(new Vector2d(1,-31))
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //SAMPLE 1 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromSpecimen(arm,slider))
                .afterTime(1.7,AutoSeq.SamplePickForSpecimen(arm,slider))
                .splineToLinearHeading(new Pose2d(new Vector2d(27.5,-32),Math.PI/12),Math.PI/4)
                .waitSeconds(0.25)
                .afterTime(0.7,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(new Vector2d(32,-45), -Math.PI/4)
                //SAMPLE 2 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .afterTime(0.85,AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(new Vector2d(38.5,-32),Math.PI/12)
                .waitSeconds(0.25)
                .afterTime(0.75,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(new Vector2d(38,-45), -Math.PI/4)
                //SAMPLE 3 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .afterTime(0.85,AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(new Vector2d(48.5,-30),Math.PI/12)
                .waitSeconds(0.25)
                .afterTime(0.65,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(new Vector2d(42,-45), -Math.PI/4)
                //FIRST SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPosFromSampleDrop(arm,slider))
                .waitSeconds(0.45)
                .splineToLinearHeading(new Pose2d(new Vector2d(35.5,-63),-Math.PI/2),3*Math.PI/2)
                .stopAndAdd(AutoSeq.SpecimenPick(arm,slider))
                .strafeToConstantHeading(new Vector2d(4,-30))
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //SECOND SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPos(arm,slider))
                .splineToConstantHeading(new Vector2d(36,-64),-Math.PI/2)
                .stopAndAdd(AutoSeq.SpecimenPick(arm,slider))
                .strafeToConstantHeading(new Vector2d(5,-30))
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //THIRD SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPos(arm,slider))
                .splineToConstantHeading(new Vector2d(36,-64),-Math.PI/2)
                .stopAndAdd(AutoSeq.SpecimenPick(arm,slider))
                .strafeToConstantHeading(new Vector2d(8,-30))
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //PARKING
                .afterTime(0.01,AutoSeq.TeleOpInit(arm,slider))
                .strafeToConstantHeading(new Vector2d(45,-60),baseConst)
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
