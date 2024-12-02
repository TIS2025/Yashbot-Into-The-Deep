package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Sequences.FinalAutoSeq;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Slider Extension Test")
public class SliderTest extends LinearOpMode {
    static List<Action> ftc = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();

        new InitSeq(arm,hanger,slider);

        waitForStart();
        while (opModeIsActive()){
            ftc = updateAction();
            if(gamepad1.a){
                ftc.add(FinalAutoSeq.SamplePickPosFullExtRightYaw(arm,slider));
            }
            if(gamepad1.b){
                ftc.add(FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider));
            }

            if(gamepad1.x){
            }
            if(gamepad1.y){
                slider.setTurret(1250);
            }
            if(gamepad1.dpad_down){
                ftc.add(FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider));
            }
            if(gamepad1.dpad_up){
                ftc.add(FinalAutoSeq.SampleDropPosNoYaw0Ext(arm,slider));
            }
            if(gamepad1.dpad_right){
                ftc.add(FinalAutoSeq.SampleDrop(arm,slider));
            }
            if(gamepad1.dpad_left){
                ftc.add(FinalAutoSeq.SamplePick(arm,slider));
            }
            if(gamepad1.left_bumper){
                ftc.add(FinalAutoSeq.SampleDropPosYawLeft0Ext(arm,slider));
            }
            if(gamepad1.right_bumper){
                ftc.add(FinalAutoSeq.SampleDropPosYawRightMaxExt(arm,slider));
            }

            telemetry.addData("Turret Pos",robot.turret.getCurrentPosition());
            telemetry.addData("Ext left Pos",robot.extLeft.getCurrentPosition());
            telemetry.addData("Ext right Pos",robot.extRight.getCurrentPosition());
            telemetry.addData("Hanger Pos",robot.hanger.getCurrentPosition());
            telemetry.addData("Shoulder",robot.shoulder.getPosition());
            telemetry.addData("Yaw",robot.yaw.getPosition());
            telemetry.addData("Elbow",robot.elbow.getPosition());
            telemetry.addData("Wrist",robot.wrist.getPosition());
            telemetry.addData("Gripper",robot.gripper.getPosition());
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
