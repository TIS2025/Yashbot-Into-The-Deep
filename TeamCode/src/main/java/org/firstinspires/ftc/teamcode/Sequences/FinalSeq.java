package org.firstinspires.ftc.teamcode.Sequences;

import static org.firstinspires.ftc.teamcode.Globals.MotorConst.extTimeConst;
import static org.firstinspires.ftc.teamcode.Globals.MotorConst.turretTimeConst;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.sun.tools.javac.util.SharedNameTable;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorKLNavxMicro;
import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class FinalSeq {



    public static Action HomePos(Arm arm, Slider slider) {

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(() -> slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.HOME)),
//                new SleepAction(0.3),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.HOME)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(extTime),
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN))
        );
    }

    public static Action SamplePickPos(Arm arm){

        return new SequentialAction(
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SamplePick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN))
        );
    }

    public static Action SampleDropPos(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;
        double turTime = (double) Math.abs(slider.TurretPos() - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(turTime),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP))
        );
    }


    public static Action SampleDrop(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.HOME)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN))
        );
    }

    public static Action SampleDropObsZone(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extHorizontalMax) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1)
        );
    }

    public static Action SpecimenPickPos(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extSpecimenPrePick) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new SleepAction(extTime + 0.1),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action SpecimenPick(Arm arm, Slider slider){

        double turTime = (double) Math.abs(slider.TurretPos() - MotorConst.turretSpecimenPreDrop)/MotorConst.turretDown * turretTimeConst;

        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.15),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PICK)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_DROP)),
                new SleepAction(turTime),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0))
        );
    }

    public static Action SpecimenDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.45),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.15),
                SpecimenPickPos(arm,slider)
        );
    }

    public static Action HighHang1(Arm arm, Slider slider, Hanger hanger){
        return new SequentialAction(
                new InstantAction(()-> hanger.updateHangerState(Hanger.HangerState.PHASE1_UP)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.PRE_HANG)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_HANG)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_HANG)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.PRE_HANG)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(0.5),
                //TODO EXT
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action HighHang2(Slider slider, Hanger hanger){
        return new SequentialAction(
                new InstantAction(()-> hanger.updateHangerState(Hanger.HangerState.PHASE1_DOWN)),
                new SleepAction(2),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(2),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HIGH_HANG))
        );
    }

}
