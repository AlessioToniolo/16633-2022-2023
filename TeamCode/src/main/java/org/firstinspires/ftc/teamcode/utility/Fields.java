package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
public class Fields {
    // Use `public static`

    //Slider
    public static int sliderMinimumTarget = 0;
    public static int sliderMaximumTarget=1200;

    public static int armMinimumTarget = 0;
    public static int armMaximumTarget = 2000;

    public static int armDepositForward=546;
    public static int armDepositBackwards=1761;
    public static int armPickup = 0;

    //refrence variables
    public static int referenceGroundPickup =0;
    public static int referenceConeStackPickup=1;
    public static int referenceLowJunction = 2;
    public static int referenceMiddleJunction = 3;
    public static int referenceHighJunction = 4;

    /**
    public static int armIntakeLevel=0;
    public static int armGroundJunctionLevel=0;
    public static int armLowJunctionLevel=540;
    public static int armMidJunctionLevel=0;
    public static int armHighJunctionLevel=0;**/

    public static int sliderGroundPickup=0;
    public static int sliderConeStackPickup=0;
    public static int sliderLowJunctionLevel=0;
    public static int sliderMidJunctionLevel=0;
    public static int sliderHighJunctionLevel=0;

}