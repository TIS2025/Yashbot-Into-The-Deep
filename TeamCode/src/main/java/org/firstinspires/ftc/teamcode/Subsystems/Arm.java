package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Arm {
    private RobotHardware robot;
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
        AUTO_PICK_RIGHT
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
        SPECIMEN_PRE_INTAKE,
        SPECIMEN_INTAKE,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        AUTO_INIT
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
        SPECIMEN_PRE_INTAKE,
        SPECIMEN_INTAKE,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        AUTO_INIT
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
        AUTO_PICK_LEFT

    }

    public GripperState gripperState = GripperState.INIT;
    public WristState wristState = WristState.INIT;
    public ElbowState elbowState = ElbowState.INIT;
    public ShoulderState shoulderState = ShoulderState.INIT;
    public YawState yawState = YawState.INIT;

    public void updateGripperState(GripperState state){
        switch(state) {
            case INIT:
                setGripper(ServoConst.gripperInit);
                gripperState = GripperState.INIT;
                break;
            case CLOSE:
                setGripper(ServoConst.gripperClose);
                gripperState = GripperState.CLOSE;
                break;
            case OPEN:
                setGripper(ServoConst.gripperOpen);
                gripperState = GripperState.OPEN;
                break;
        }
    }

    public void updateWristState(WristState state){
        switch (state){
            case INIT:
                setWrist(ServoConst.wristInit);
                wristState = WristState.INIT;
                break;
            case WRIST0:
                setWrist(ServoConst.wrist0);
                wristState = WristState.WRIST0;
                break;
            case WRIST90:
                setWrist(ServoConst.wrist90);
                wristState = WristState.WRIST90;
                break;
            case WRIST180:
                setWrist(ServoConst.wrist180);
                wristState = WristState.WRIST180;
                break;
            case AUTO_PICK_RIGHT:
                setWrist(ServoConst.wristAutoPickRight);
                wristState = WristState.AUTO_PICK_RIGHT;
                break;
            case AUTO_PICK_LEFT:
                setWrist(ServoConst.wristAutoPickLeft);
                wristState = WristState.AUTO_PICK_LEFT;
                break;
        }
    }

    public void updateElbowState(ElbowState state){
        switch (state){
            case INIT:
                setElbow(ServoConst.elbowInit);
                elbowState = ElbowState.INIT;
                break;
            case UP:
                setElbow(ServoConst.elbowUp);
                elbowState = ElbowState.UP;
                break;
            case DOWN:
                setElbow(ServoConst.elbowDown);
                elbowState = ElbowState.DOWN;
                break;
            case HOME:
                setElbow(ServoConst.elbowHome);
                elbowState = ElbowState.HOME;
                break;
            case PRE_INTAKE:
                setElbow(ServoConst.elbowPreIntake);
                elbowState = ElbowState.PRE_INTAKE;
                break;
            case INTAKE:
                setElbow(ServoConst.elbowIntake);
                elbowState = ElbowState.INTAKE;
                break;
            case POST_INTAKE:
                setElbow(ServoConst.elbowPostIntake);
                elbowState = ElbowState.POST_INTAKE;
                break;
            case PRE_BUCKET_DROP:
                setElbow(ServoConst.elbowPreBucketDrop);
                elbowState = ElbowState.PRE_BUCKET_DROP;
                break;
            case BUCKET_DROP:
                setElbow(ServoConst.elbowBucketDrop);
                elbowState = ElbowState.BUCKET_DROP;
                break;
            case SPECIMEN_PRE_INTAKE:
                setElbow(ServoConst.elbowSpecimenPrePick);
                elbowState = ElbowState.SPECIMEN_PRE_INTAKE;
                break;
            case SPECIMEN_INTAKE:
                setElbow(ServoConst.elbowSpecimenPick);
                elbowState = ElbowState.SPECIMEN_INTAKE;
                break;
            case SPECIMEN_PRE_DROP:
                setElbow(ServoConst.elbowSpecimenPreDrop);
                elbowState = ElbowState.SPECIMEN_PRE_DROP;
                break;
            case SPECIMEN_DROP:
                setElbow(ServoConst.elbowSpecimenDrop);
                elbowState = ElbowState.SPECIMEN_DROP;
                break;
            case AUTO_INIT:
                setElbow(ServoConst.elbowAutoInit);
                elbowState = ElbowState.AUTO_INIT;
                break;
        }
    }

    public void updateShoulderState(ShoulderState state){
        switch (state){
            case INIT:
                setShoulder(ServoConst.shoulderInit);
                shoulderState = ShoulderState.INIT;
                break;
            case DOWN:
                setShoulder(ServoConst.shoulderDown);
                shoulderState = ShoulderState.DOWN;
                break;
            case UP:
                setShoulder(ServoConst.shoulderUp);
                shoulderState = ShoulderState.UP;
                break;
            case HOME:
                setShoulder(ServoConst.shoulderHome);
                shoulderState = ShoulderState.HOME;
                break;
            case PRE_INTAKE:
                setShoulder(ServoConst.shoulderPreIntake);
                shoulderState = ShoulderState.PRE_INTAKE;
                break;
            case INTAKE:
                setShoulder(ServoConst.shoulderIntake);
                shoulderState = ShoulderState.INTAKE;
                break;
            case POST_INTAKE:
                setShoulder(ServoConst.shoulderPostIntake);
                shoulderState = ShoulderState.POST_INTAKE;
                break;
            case PRE_BUCKET_DROP:
                setShoulder(ServoConst.shoulderPreBucketDrop);
                shoulderState = ShoulderState.PRE_BUCKET_DROP;
                break;
            case BUCKET_DROP:
                setShoulder(ServoConst.shoulderBucketDrop);
                shoulderState = ShoulderState.BUCKET_DROP;
                break;
            case SPECIMEN_PRE_INTAKE:
                setShoulder(ServoConst.shoulderSpecimenPrePick);
                shoulderState = ShoulderState.SPECIMEN_PRE_INTAKE;
                break;
            case SPECIMEN_INTAKE:
                setShoulder(ServoConst.shoulderSpecimenPick);
                shoulderState = ShoulderState.SPECIMEN_INTAKE;
                break;
            case SPECIMEN_PRE_DROP:
                setShoulder(ServoConst.shoulderSpecimenPreDrop);
                elbowState = ElbowState.SPECIMEN_PRE_DROP;
                break;
            case SPECIMEN_DROP:
                setShoulder(ServoConst.shoulderSpecimenDrop);
                elbowState = ElbowState.SPECIMEN_DROP;
                break;
            case AUTO_INIT:
                setShoulder(ServoConst.shoulderAutoInit);
                elbowState = ElbowState.AUTO_INIT;
                break;
        }
    }

    public void updateYawState(YawState state){
        switch (state){
            case INIT:
                setYaw(ServoConst.yawInit);
                yawState = YawState.INIT;
                break;
            case NEUTRAL:
                setYaw(ServoConst.yawNeutral);
                yawState = YawState.NEUTRAL;
                break;
            case LEFT:
                setYaw(ServoConst.yawLeft);
                yawState = YawState.LEFT;
                break;
            case RIGHT:
                setYaw(ServoConst.yawRight);
                yawState = YawState.RIGHT;
                break;
            case HOME:
                setYaw(ServoConst.yawHome);
                yawState = YawState.HOME;
                break;
            case AUTO_INIT:
                setYaw(ServoConst.yawAutoInit);
                yawState = YawState.AUTO_INIT;
                break;
            case AUTO_PICK_RIGHT:
                setYaw(ServoConst.yawAutoPickRight);
                yawState = YawState.AUTO_PICK_RIGHT;
                break;
            case AUTO_SAMPLE_DROP_RIGHT:
                setYaw(ServoConst.yawAutoSampleDropRight);
                yawState = YawState.AUTO_SAMPLE_DROP_RIGHT;
                break;
            case AUTO_SAMPLE_DROP_LEFT:
                setYaw(ServoConst.yawAutoSampleDropLeft);
                yawState = YawState.AUTO_SAMPLE_DROP_LEFT;
                break;
            case AUTO_PICK_LEFT:
                setYaw(ServoConst.yawAutoPickLeft);
                yawState = YawState.AUTO_PICK_LEFT;
                break;
        }
    }

    public void setGripper(double pos){robot.gripper.setPosition(pos);}
    public void setWrist(double pos){robot.wrist.setPosition(pos);}
    public void setElbow(double pos){robot.elbow.setPosition(pos);}
    public void setShoulder(double pos){robot.shoulder.setPosition(pos);}
    public void setYaw(double pos){robot.yaw.setPosition(pos);}
}
