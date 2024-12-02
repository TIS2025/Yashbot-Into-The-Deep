package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Slider {
    private final RobotHardware robot;
    public Slider(RobotHardware robot){this.robot = robot;}

    public enum ExtState{
        MIN,
        MAX,
        MID,
        INIT,
        BUCKET_DROP,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP
    }
    public enum TurretState{
        UP,
        DOWN,
        INIT,
        PRE_BUCKET_DROP,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP
    }

    public ExtState extState = ExtState.INIT;
    public TurretState turretState = TurretState.INIT;

    public void updateExtState(@NonNull ExtState state){
        switch (state){
            case INIT:
                setExt(MotorConst.extInit);
                break;
            case MIN:
                setExt(MotorConst.extMin);
                break;
            case MID:
                setExt(MotorConst.extHorizontalMax);
                break;
            case MAX:
                setExt(MotorConst.extMax);
                break;
            case BUCKET_DROP:
                setExt(MotorConst.extHighBucketDrop);
                break;
            case SPECIMEN_PRE_PICK:
                setExt(MotorConst.extSpecimenPrePick);
                break;
            case SPECIMEN_PRE_DROP:
                setExt(MotorConst.extSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setExt(MotorConst.extSpecimenDrop);
                break;
        }
        extState = state;
    }

    public void updateTurretState(@NonNull TurretState state){
        switch (state){
            case INIT:
                setTurret(MotorConst.turretInit);
                break;
            case UP:
                setTurret(MotorConst.turretUp);
                break;
            case DOWN:
                setTurret(MotorConst.turretDown);
                break;
            case PRE_BUCKET_DROP:
                setTurret(MotorConst.turretBucketPreDrop);
                break;
            case SPECIMEN_PRE_PICK:
                setTurret(MotorConst.turretSpecimenPrePick);
                break;
            case SPECIMEN_PRE_DROP:
                setTurret(MotorConst.turretSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setTurret(MotorConst.turretSpecimenDrop);
                break;
        }
        turretState = state;
    }

    public void setExt(int target){
        int bottom_clip = (int)(robot.turret.getCurrentPosition()/1250*160);
        int target_clip = Math.max(Math.min(target - bottom_clip,3000),-bottom_clip);
        robot.extLeft.setTargetPosition(target);
        robot.extRight.setTargetPosition(target);
        robot.extLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extLeft.setPower(MotorConst.extPower);
        robot.extRight.setPower(MotorConst.extPower);
    }

    public void setTurret(int target){
        target = Math.max(Math.min(target,1300),0);
        robot.turret.setTargetPosition(target);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(MotorConst.turretPower);
    }

    public boolean isExtBusy(){
        return robot.extRight.isBusy();
    }

    public boolean isTurretBusy(){
        return robot.turret.isBusy();
    }
}
