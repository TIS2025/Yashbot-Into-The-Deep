package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class AutoSeq {
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

    public static Action SampleDropYawRight(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }

    public static Action SampleDropYawLeft(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_LEFT)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }

    public static Action SampleDropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP))
        );
    }
    public static Action SamplePickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MID)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SamplePickYaw(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_FIRST_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.AUTO_FIRST_PICK)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.5),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP))

        );
    }

    public static Action SamplePick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP))

        );
    }

    public static Action HomePos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN))
        );
    }
}