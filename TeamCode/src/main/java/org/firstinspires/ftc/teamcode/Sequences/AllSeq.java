package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class AllSeq {
    public static Action HomePos(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action InitPos(Slider slider, Arm arm, Hanger hanger){
        return new SequentialAction();
    }

    public static Action SamplePickPos(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SamplePick(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SampleDropPos(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SampleDrop(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SpecimenPickPos(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SpecimenPick(Slider slider, Arm arm){
        return new SequentialAction();
    }

    public static Action SpecimenDrop(Slider slider, Arm arm){
        return new SequentialAction();
    }


}
