package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Sequences.FinalSeq;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Sequences.IntakeSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sequence Test")
public class TestSequences extends LinearOpMode {
    static List<Action> ftc = new ArrayList<>();

    int specimenState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        int hanger_pos = 0;

        boolean wrist_rotate = false;
        double distance;

        robot.reset_encoders();
        new InitSeq(arm,hanger,slider);
        waitForStart();
        while(opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);
            P2.copy(C2);
            C2.copy(gamepad2);
            distance = robot.colorSensor.getDistance(DistanceUnit.MM);

            robot.drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x*0.5
                    )
            );
            ftc = updateAction();
            if(gamepad1.a){
//                ftc.add(IntakeSeq.Home(arm,slider));
                ftc.add(FinalSeq.HomePos(arm,slider));
            }
            if(gamepad1.b){
//                ftc.add(IntakeSeq.PreSampleIntake(arm));
                ftc.add(FinalSeq.SamplePickPos(arm));
            }
            if(gamepad1.x){
//                ftc.add(
//                        new SequentialAction(
//                                IntakeSeq.SampleIntake(arm),
//                                new SleepAction(1),
//                                IntakeSeq.PostSampleIntake(arm)
//                        )
//                );
                ftc.add(FinalSeq.SamplePick(arm,slider));
            }
            if(C1.y && !P1.y && specimenState == 0){
                ftc.add(IntakeSeq.PreSpecimenIntake(arm,slider));
                specimenState = 1;
            }

            if(C1.y && !P1.y && specimenState == 1){
                ftc.add(
                        new SequentialAction(
                                IntakeSeq.SpecimenIntake(arm),
                                new SleepAction(0.3),
                                IntakeSeq.SpecimenPreDrop(arm,slider)
                        )
                );
                specimenState = 2;
            }
            if(gamepad1.dpad_up){
//                ftc.add(BucketSeq.PreDrop(slider,arm));
                ftc.add(FinalSeq.SampleDropPos(arm,slider));
            }
            if(gamepad1.dpad_right){
//                ftc.add(BucketSeq.Drop(slider,arm));
                ftc.add(FinalSeq.SampleDrop(arm,slider));
            }
            if(C1.y && !P1.y && specimenState == 2){
                ftc.add(IntakeSeq.SpecimenDrop(arm,slider));
                specimenState = 0;
            }
            if(C2.left_bumper && !P2.left_bumper) slider.setExt(robot.extLeft.getCurrentPosition()+100);
            if(C2.right_bumper && !P2.right_bumper) slider.setExt(robot.extLeft.getCurrentPosition()-100);

            if(C2.a && !P2.a){
                wrist_rotate = !wrist_rotate;
                arm.updateWristState(wrist_rotate? Arm.WristState.WRIST0: Arm.WristState.WRIST90);
            }


            if(C2.dpad_up && !P2.dpad_up) hanger_pos+=50;
            if(C2.dpad_down && !P2.dpad_down) hanger_pos-=50;

//            if(C2.left_bumper) hanger.setHanger(1100);
//            if(C2.right_bumper) hanger.setHanger(400);

//            hanger.setHanger(Math.max(hanger_pos,0));




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
            telemetry.addData("intake_state",specimenState);
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
