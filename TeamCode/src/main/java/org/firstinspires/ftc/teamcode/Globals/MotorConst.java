package org.firstinspires.ftc.teamcode.Globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorConst {

    public static int extInit = 0;
    public static int extMin = 0;
    public static int extMax = 2900;
    public static int extHorizontalMax = 1200;
    public static int extHighBucketDrop = 2750;
    public static int extSpecimenPrePick = 0;
    public static int extSpecimenPreDrop = 700;
    public static int extSpecimenDrop = 1200;
    public static int extHighHang = 1200;
    public static int extFirstSpecimen = 500;

    public static int turretInit = 0;
    public static int turretUp = 0;
    public static int turretDown = 1250;
    public static int turretBucketPreDrop = 0;
    public static int turretSpecimenPrePick = 300;
    public static int turretSpecimenPreDrop = 0;
    public static int turretSpecimenDrop = 0;
    public static int turretPreHang = 190;

    public static int hangerInit = 0;
    public static int hangerUpPhase1 = 1100;
    public static int hangerDownPhase1 = 400;

    public static double extPower = 1;
    public static double turretPower = 0.8;

    public static double turretTimeConst = 0.55;
    public static double extTimeConst = 1.3;
}
