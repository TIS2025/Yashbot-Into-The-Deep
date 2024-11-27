package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class IntakeSeq {
    public static Action Home(Arm arm, Slider slider) {
        return new ParallelAction(
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.HOME)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action PreSampleIntake(Arm arm){
        return new ParallelAction(
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SampleIntake(Arm arm){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action PostSampleIntake(Arm arm){
        return new ParallelAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE))
        );
    }

    public static Action PreSpecimenIntake(Arm arm, Slider slider){
        return new ParallelAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_INTAKE)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action SpecimenIntake(Arm arm){
        return new ParallelAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_INTAKE))
        );
    }

    public static Action SpecimenPreDrop(Arm arm,Slider slider){
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_DROP)),
                        new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_DROP)),
                        new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                        new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_DROP))
                ),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_DROP))
        );
    }

    public static Action SpecimenDrop(Arm arm, Slider slider){
        return new SequentialAction(
//                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_DROP)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }
}