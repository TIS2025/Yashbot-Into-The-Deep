package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.FinalSeq;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "TeleOp", name = "Basic TeleOp Educo")
public class TeleOp_Educo extends LinearOpMode {
    static List<Action> ftc = new ArrayList<>();
    MecanumDrive drive;

    enum BotState{
        SAMPLE_MODE,
        SPECIMEN_MODE
    }

    enum SampleState{
        HOME_POS,
        PICK_POS,
        PICK,
        DROP_POS,
    }

    enum SpecimenState{
        PICK_POS,
        DROP_POS,
    }

    SampleState sampleState = SampleState.HOME_POS;
    SpecimenState specimenState = SpecimenState.PICK_POS;
    BotState intakeState = BotState.SAMPLE_MODE;

    //FLAGS
    boolean HANGER_FLAG2 = false;
    boolean HANGER_FLAG1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        int slider_pos = 0;

        boolean wrist_rotate = true;
        double distance;
        double drive_coeff = 1;

        while (opModeInInit()){
            P2.copy(C2);
            C2.copy(gamepad2);

            if(C2.left_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() - 25);
            }
            if(C2.right_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() + 25);
            }
        }

        robot.reset_encoders();
        new InitSeq(arm,hanger,slider);
        waitForStart();
        while(opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);
            P2.copy(C2);
            C2.copy(gamepad2);
            distance = robot.colorSensor.getDistance(DistanceUnit.MM);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -C1.left_stick_y*drive_coeff,
                                    -C1.left_stick_x*drive_coeff
                            ),
                            -C1.right_stick_x*0.5
                    )
            );
            ftc = updateAction();

            if(C1.left_trigger>0.5) drive_coeff = 0.25;
            else drive_coeff = 1;

            if(C2.dpad_down && C1.dpad_down){
                HANGER_FLAG2 = true;
            }

            if(C1.dpad_down){
                HANGER_FLAG1 = true;
            }

            if(C2.dpad_left && !P2.dpad_left && specimenState == SpecimenState.PICK_POS){
                intakeState = BotState.SAMPLE_MODE;
                sampleState = SampleState.HOME_POS;
                ftc.add(FinalSeq.HomePos(arm,slider));
            }

            if(C2.dpad_right && !P2.dpad_right && sampleState == SampleState.HOME_POS){
                specimenState = SpecimenState.PICK_POS;
                intakeState = BotState.SPECIMEN_MODE;
                ftc.add(FinalSeq.SpecimenPickPos(arm,slider));
            }

            if(C1.a && !P1.a && intakeState == BotState.SAMPLE_MODE && (sampleState == SampleState.HOME_POS || sampleState == SampleState.PICK_POS || sampleState == SampleState.PICK)){
                ftc.add(FinalSeq.HomePos(arm,slider));
                sampleState = SampleState.HOME_POS;
            }

            if(C1.b && !P1.b && intakeState == BotState.SAMPLE_MODE && (sampleState == SampleState.HOME_POS || sampleState == SampleState.PICK)){
                ftc.add(FinalSeq.SamplePickPos(arm));
                sampleState = SampleState.PICK_POS;
            }

            if(C1.x && !P1.x && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                ftc.add(FinalSeq.SamplePick(arm,slider));
                sampleState = SampleState.PICK;
            }

            if(C1.left_bumper && !P1.left_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK){
                ftc.add(FinalSeq.SampleDropPos(arm,slider));
                sampleState = SampleState.DROP_POS;
            }

            if(C1.right_bumper && !P1.right_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.DROP_POS){
                ftc.add(FinalSeq.SampleDrop(arm,slider));
                sampleState = SampleState.HOME_POS;
            }

            if(C1.right_bumper && !P1.right_bumper && intakeState == BotState.SPECIMEN_MODE && specimenState == SpecimenState.DROP_POS){
                ftc.add(FinalSeq.SpecimenDrop(arm,slider));
                specimenState = SpecimenState.PICK_POS;
            }
            if(C1.left_bumper && !P1.left_bumper && intakeState == BotState.SPECIMEN_MODE && specimenState == SpecimenState.PICK_POS) {
                ftc.add(FinalSeq.SpecimenPick(arm,slider));
                specimenState = SpecimenState.DROP_POS;
            }

            if(C2.left_bumper && !P2.left_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                slider_pos+=400;
                slider_pos = Math.min(slider_pos,1200);
                slider.setExt(slider_pos);
            }
            if(C2.right_bumper && !P2.right_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                slider_pos-=400;
                slider_pos = Math.max(slider_pos,0);
                slider.setExt(slider_pos);
            }

            if(C2.a && !P2.a && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                wrist_rotate = !wrist_rotate;
                arm.updateWristState(wrist_rotate? Arm.WristState.WRIST0: Arm.WristState.WRIST90);
            }

            if(C1.left_trigger>0.75 && !(P1.left_trigger>0.75) && HANGER_FLAG1) hanger.setHanger(1100);
            if(C1.right_trigger>0.75 && !(P1.right_trigger>0.75) && HANGER_FLAG1) hanger.setHanger(500);

            if(C1.left_trigger>0.75 && !(P1.left_trigger>0.75) && HANGER_FLAG2) ftc.add(FinalSeq.HighHang1(arm, slider, hanger));
            if(C1.right_trigger>0.75 && !(P1.right_trigger>0.75) && HANGER_FLAG2) ftc.add(FinalSeq.HighHang2(slider,hanger));

            telemetry.addData("Turret Pos",robot.turret.getCurrentPosition());
            telemetry.addData("Ext left Pos",robot.extLeft.getCurrentPosition());
            telemetry.addData("Ext right Pos",robot.extRight.getCurrentPosition());
            telemetry.addData("Hanger Pos",robot.hanger.getCurrentPosition());
            telemetry.addData("Shoulder",robot.shoulder.getPosition());
            telemetry.addData("Yaw",robot.yaw.getPosition());
            telemetry.addData("Elbow",robot.elbow.getPosition());
            telemetry.addData("Wrist",robot.wrist.getPosition());
            telemetry.addData("Gripper",robot.gripper.getPosition());
            telemetry.addData("Distance",distance);
            telemetry.addData("Specimen State",specimenState);
            telemetry.addData("Sample State",sampleState);
            telemetry.addData("IntakeState",intakeState);
            telemetry.update();
        }
    }

    private static List<Action> updateAction() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : ftc) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);
        return newActions;
    }
}
