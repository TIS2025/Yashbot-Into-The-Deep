package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@TeleOp(group = "Utils", name = "Get Values")
public class GetValues extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);

        double elbowPos = 0.5;
        double shoulderPos = 0.5;
        double yawPos = 0.5;
        double wristPos = 0.5;
        double gripperPos = 0.5;
        int extPos = 0;
        int hangerPos = 0;
        int turretPos = 0;


        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        robot.reset_encoders();
        new InitSeq(arm,hanger,slider);

        waitForStart();
        while (opModeIsActive()){
            P.copy(C);
            C.copy(gamepad1);

            P2.copy(C2);
            C2.copy(gamepad2);

            if(C.a && !P.a) shoulderPos+=0.01;
            if(C.y && !P.y) shoulderPos-=0.01;

            if(C.x && !P.x) yawPos+=0.005;
            if(C.b && !P.b) yawPos-=0.005;

            if(C.dpad_up && !P.dpad_up) elbowPos+=0.005;
            if(C.dpad_down && !P.dpad_down) elbowPos-=0.005;

            if(C.dpad_left && !P.dpad_left) wristPos+=0.01;
            if(C.dpad_right && !P.dpad_right) wristPos-=0.01;

            if(C.left_stick_y>0.75) hangerPos+=5;
            if(C.left_stick_y<-0.75) hangerPos-=5;

            if(C.left_stick_x>0.75 && !(P.left_stick_x>0.75)) gripperPos+=0.05;
            if(C.left_stick_x<-0.75 && !(P.left_stick_x<-0.75)) gripperPos-=0.05;

            if(C.left_bumper && !P.left_bumper) extPos+=50;
            if(C.right_bumper && !P.right_bumper) extPos-=50;

            if(C.right_stick_y>0.75) turretPos+=5;
            if(C.right_stick_y<-0.75) turretPos-=5;

            if(C2.dpad_up) hanger.setHanger(1100);
            if(C2.dpad_right) hanger.setHanger(500);

            if(C2.dpad_down) {
                turretPos = 100;
                extPos = 2800;
            }

            if(C2.dpad_left){
                turretPos = 0;
                sleep(500);
                extPos = 300;
            }

            turretPos = Math.max(turretPos,0);
            extPos = Math.max(extPos,0);
            shoulderPos = Range.clip(shoulderPos,0,1);
            wristPos = Range.clip(wristPos,0,1);
            yawPos = Range.clip(yawPos,0,1);
            elbowPos = Range.clip(elbowPos,0,1);
            gripperPos = Range.clip(gripperPos,0,1);

            arm.setShoulder(shoulderPos);
            arm.setYaw(yawPos);
            arm.setElbow(elbowPos);
            arm.setWrist(wristPos);
            arm.setGripper(gripperPos);
            slider.setTurret(turretPos);
            slider.setExt(extPos);
//            hanger.setHanger(hangerPos);


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
}
