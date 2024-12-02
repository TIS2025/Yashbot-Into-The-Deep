package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Config
public class FinalAutoSeq {
    public static double t1=1,t2=1,t3=1,t4=1;
    public static Action Init(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(2),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action SamplePickPosNoExtNoYaw(Arm arm, Slider slider){
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0))
                ),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN))
        );
    }

    public static Action SamplePickPosNoExtLeftYaw(Arm arm, Slider slider){
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT))
                ),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_PICK_LEFT))
        );
    }

    public static Action SamplePickPosFullExtRightYaw(Arm arm, Slider slider){
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.AUTO_PICK_RIGHT))
                ),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_PICK_RIGHT)),
                new SleepAction(1),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MID))
        );
    }

    public static Action SampleDropPosNoYaw0Ext(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosNoYawMaxExt(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosYawLeft0Ext(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_LEFT)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosYawLeftMaxExt(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_LEFT)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosYawRight0Ext(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosYawRight0ExtTurretUp(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SampleDropPosYawRightMaxExt(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP))
        );
    }

    public static Action SamplePick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0))
        );
    }

    public static Action SampleDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }
}
