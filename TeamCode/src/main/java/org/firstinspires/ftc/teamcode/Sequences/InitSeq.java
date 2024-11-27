package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class InitSeq {
    public InitSeq(Arm arm, Hanger hanger, Slider slider){
        Actions.runBlocking(
                new ParallelAction(
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.INIT)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.INIT)),
                        new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.INIT)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.INIT)),
                        new InstantAction(()->hanger.updateHangerState(Hanger.HangerState.INIT)),
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                        new InstantAction(()->slider.updateTurretState(Slider.TurretState.INIT))
                )
        );
    }
}
