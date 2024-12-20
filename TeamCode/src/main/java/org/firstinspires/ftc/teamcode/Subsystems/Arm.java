package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Arm {
    private final RobotHardware robot;
    public Arm(RobotHardware robot){this.robot = robot;}

    public enum GripperState{
        OPEN,
        CLOSE,
        INIT
    }
    public enum WristState{
        WRIST0,
        WRIST90,
        INIT,
        WRIST180,
        AUTO_PICK_LEFT,
        AUTO_PICK_LEFT2,
        AUTO_PICK_RIGHT,
        SPECIMEN_PRE_PICK
    }
    public enum ElbowState{
        INIT,
        UP,
        DOWN,
        HOME,
        PRE_INTAKE,
        INTAKE,
        POST_INTAKE,
        PRE_BUCKET_DROP,
        BUCKET_DROP,
        AUTO_INIT,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        PRE_HANG
    }
    public enum ShoulderState{
        INIT,
        UP,
        DOWN,
        HOME,
        PRE_INTAKE,
        INTAKE,
        POST_INTAKE,
        PRE_BUCKET_DROP,
        BUCKET_DROP,
        AUTO_INIT,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        PRE_HANG,
        AUTO_PRE_INTAKE
    }
    public enum YawState{
        INIT,
        NEUTRAL,
        LEFT,
        RIGHT,
        HOME,
        AUTO_INIT,
        AUTO_PICK_RIGHT,
        AUTO_SAMPLE_DROP_RIGHT,
        AUTO_SAMPLE_DROP_LEFT,
        AUTO_PICK_LEFT,
        AUTO_PICK_LEFT2,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PRE_DROP,
        PRE_HANG
    }

    public GripperState gripperState = GripperState.INIT;
    public WristState wristState = WristState.INIT;
    public ElbowState elbowState = ElbowState.INIT;
    public ShoulderState shoulderState = ShoulderState.INIT;
    public YawState yawState = YawState.INIT;

    public void updateGripperState(@NonNull GripperState state){
        switch(state) {
            case INIT:
                setGripper(ServoConst.gripperInit);
                break;
            case CLOSE:
                setGripper(ServoConst.gripperClose);
                break;
            case OPEN:
                setGripper(ServoConst.gripperOpen);
                break;
        }
        gripperState = state;
    }

    public void updateWristState(@NonNull WristState state){
        switch (state){
            case INIT:
                setWrist(ServoConst.wristInit);
                break;
            case WRIST0:
                setWrist(ServoConst.wrist0);
                break;
            case WRIST90:
                setWrist(ServoConst.wrist90);
                break;
            case WRIST180:
                setWrist(ServoConst.wrist180);
                break;
            case AUTO_PICK_RIGHT:
                setWrist(ServoConst.wristAutoPickRight);
                break;
            case AUTO_PICK_LEFT:
                setWrist(ServoConst.wristAutoPickLeft);
                break;
            case SPECIMEN_PRE_PICK:
                setWrist(ServoConst.wristSpecimenPrePick);
                break;
            case AUTO_PICK_LEFT2:
                setWrist(ServoConst.wristAutoPickLeft2);
                break;
        }
        wristState = state;
    }

    public void updateElbowState(@NonNull ElbowState state){
        switch (state){
            case INIT:
                setElbow(ServoConst.elbowInit);
                break;
            case UP:
                setElbow(ServoConst.elbowUp);
                break;
            case DOWN:
                setElbow(ServoConst.elbowDown);
                break;
            case HOME:
                setElbow(ServoConst.elbowHome);
                break;
            case PRE_INTAKE:
                setElbow(ServoConst.elbowPreIntake);
                break;
            case INTAKE:
                setElbow(ServoConst.elbowIntake);
                break;
            case POST_INTAKE:
                setElbow(ServoConst.elbowPostIntake);
                break;
            case PRE_BUCKET_DROP:
                setElbow(ServoConst.elbowPreBucketDrop);
                break;
            case BUCKET_DROP:
                setElbow(ServoConst.elbowBucketDrop);
                break;
            case AUTO_INIT:
                setElbow(ServoConst.elbowAutoInit);
                break;
            case SPECIMEN_PRE_PICK:
                setElbow(ServoConst.elbowSpecimenPrePick);
                break;
            case SPECIMEN_PICK:
                setElbow(ServoConst.elbowSpecimenPick);
                break;
            case SPECIMEN_PRE_DROP:
                setElbow(ServoConst.elbowSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setElbow(ServoConst.elbowSpecimenDrop);
                break;
            case PRE_HANG:
                setElbow(ServoConst.elbowPreHang);
                break;
        }
        elbowState = state;
    }

    public void updateShoulderState(@NonNull ShoulderState state){
        switch (state){
            case INIT:
                setShoulder(ServoConst.shoulderInit);
                break;
            case DOWN:
                setShoulder(ServoConst.shoulderDown);
                break;
            case UP:
                setShoulder(ServoConst.shoulderUp);
                break;
            case HOME:
                setShoulder(ServoConst.shoulderHome);
                break;
            case PRE_INTAKE:
                setShoulder(ServoConst.shoulderPreIntake);
                break;
            case INTAKE:
                setShoulder(ServoConst.shoulderIntake);
                break;
            case POST_INTAKE:
                setShoulder(ServoConst.shoulderPostIntake);
                break;
            case PRE_BUCKET_DROP:
                setShoulder(ServoConst.shoulderPreBucketDrop);
                break;
            case BUCKET_DROP:
                setShoulder(ServoConst.shoulderBucketDrop);
                break;
            case AUTO_INIT:
                setShoulder(ServoConst.shoulderAutoInit);
                break;
            case SPECIMEN_PRE_PICK:
                setShoulder(ServoConst.shoulderSpecimenPrePick);
                break;
            case SPECIMEN_PICK:
                setShoulder(ServoConst.shoulderSpecimenPick);
                break;
            case SPECIMEN_PRE_DROP:
                setShoulder(ServoConst.shoulderSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setShoulder(ServoConst.shoulderSpecimenDrop);
                break;
            case PRE_HANG:
                setShoulder(ServoConst.shoulderPreHang);
                break;
            case AUTO_PRE_INTAKE:
                setShoulder(ServoConst.shoulderAutoPreIntake);
                break;
        }
        shoulderState = state;
    }

    public void updateYawState(@NonNull YawState state){
        switch (state){
            case INIT:
                setYaw(ServoConst.yawInit);
                break;
            case NEUTRAL:
                setYaw(ServoConst.yawNeutral);
                break;
            case LEFT:
                setYaw(ServoConst.yawLeft);
                break;
            case RIGHT:
                setYaw(ServoConst.yawRight);
                break;
            case HOME:
                setYaw(ServoConst.yawHome);
                break;
            case AUTO_INIT:
                setYaw(ServoConst.yawAutoInit);
                break;
            case AUTO_PICK_RIGHT:
                setYaw(ServoConst.yawAutoPickRight);
                break;
            case AUTO_SAMPLE_DROP_RIGHT:
                setYaw(ServoConst.yawAutoSampleDropRight);
                break;
            case AUTO_SAMPLE_DROP_LEFT:
                setYaw(ServoConst.yawAutoSampleDropLeft);
                break;
            case AUTO_PICK_LEFT:
                setYaw(ServoConst.yawAutoPickLeft);
                break;
            case SPECIMEN_PRE_PICK:
                setYaw(ServoConst.yawSpecimenPrePick);
                break;
            case SPECIMEN_PRE_DROP:
                setYaw(ServoConst.yawSpecimenPreDrop);
                break;
            case PRE_HANG:
                setYaw(ServoConst.yawPreHang);
                break;
            case AUTO_PICK_LEFT2:
                setYaw(ServoConst.yawAutoPickLeft2);
                break;
        }
        yawState = state;
    }

    public void setGripper(double pos){robot.gripper.setPosition(pos);}
    public void setWrist(double pos){robot.wrist.setPosition(pos);}
    public void setElbow(double pos){robot.elbow.setPosition(pos);}
    public void setShoulder(double pos){robot.shoulder.setPosition(pos);}
    public void setYaw(double pos){robot.yaw.setPosition(pos);}
}
