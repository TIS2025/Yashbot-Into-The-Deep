package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Slider {
    private RobotHardware robot;
    public Slider(RobotHardware robot){this.robot = robot;}

    public enum ExtState{
        MIN,MAX,MID,INIT,BUCKET_DROP,SPECIMEN_PRE_PICK,SPECIMEN_PRE_DROP,SPECIMEN_DROP
    }
    public enum TurretState{
        UP,DOWN,INIT,PRE_BUCKET_DROP,SPECIMEN_PRE_PICK,SPECIMEN_PRE_DROP,SPECIMEN_DROP
    }

    int extOffset = 0;

    public ExtState extState = ExtState.INIT;
    public TurretState turretState = TurretState.INIT;

    public void updateExtState(ExtState state){
        switch (state){
            case INIT:
                setExt(MotorConst.extInit);
                extState = ExtState.INIT;
                break;
            case MIN:
                setExt(MotorConst.extMin);
                extState = ExtState.MIN;
                break;
            case MID:
                setExt(MotorConst.extMid);
                extState = ExtState.MID;
                break;
            case MAX:
                setExt(MotorConst.extMax);
                extState = ExtState.MAX;
                break;
            case BUCKET_DROP:
                setExt(MotorConst.extHighBucketDrop);
                extState = ExtState.BUCKET_DROP;
                break;
            case SPECIMEN_PRE_PICK:
                setExt(MotorConst.extSpecimenPrePick);
                extState = ExtState.SPECIMEN_PRE_PICK;
                break;
            case SPECIMEN_PRE_DROP:
                setExt(MotorConst.extSpecimenPreDrop);
                extState = ExtState.SPECIMEN_PRE_DROP;
                break;
            case SPECIMEN_DROP:
                setExt(MotorConst.extSpecimenDrop);
                extState = ExtState.SPECIMEN_DROP;
                break;
        }
    }

    public void updateTurretState(TurretState state){
        switch (state){
            case INIT:
                setTurret(MotorConst.turretInit);
                turretState = TurretState.INIT;
                break;
            case UP:
                setTurret(MotorConst.turretUp);
                turretState = TurretState.UP;
                break;
            case DOWN:
                setTurret(MotorConst.turretDown);
                turretState = TurretState.DOWN;
                break;
            case PRE_BUCKET_DROP:
                setTurret(MotorConst.turretBucketPreDrop);
                turretState = TurretState.PRE_BUCKET_DROP;
                break;
            case SPECIMEN_PRE_PICK:
                setTurret(MotorConst.turretSpecimenPrePick);
                turretState = TurretState.SPECIMEN_PRE_PICK;
                break;
            case SPECIMEN_PRE_DROP:
                setTurret(MotorConst.turretSpecimenPreDrop);
                turretState = TurretState.SPECIMEN_PRE_DROP;
                break;
            case SPECIMEN_DROP:
                setTurret(MotorConst.turretSpecimenDrop);
                turretState = TurretState.SPECIMEN_DROP;
                break;
        }
    }

    public void setExt(int target){
        int bottom_clip = (int)(robot.turret.getCurrentPosition()/1250*160);
        int target_clip = Math.max(Math.min(target - bottom_clip,3000),-bottom_clip);
        robot.extLeft.setTargetPosition(target_clip);
        robot.extRight.setTargetPosition(target_clip);
        robot.extLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extLeft.setPower(1);
        robot.extRight.setPower(1);
    }

    public boolean isExtBusy(){
        return robot.extRight.isBusy();
    }

    public boolean isTurretBusy(){
        return robot.turret.isBusy();
    }

    public void setTurret(int target){
        robot.turret.setTargetPosition(target);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.35);
    }
}
