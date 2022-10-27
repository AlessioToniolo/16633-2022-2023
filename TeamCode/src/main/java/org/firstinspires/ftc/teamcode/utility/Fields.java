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
    //slider
    public static int referenceGroundPickup =0;
    public static int referenceConeStackPickup=1;
    public static int referenceLowJunction = 2;
    public static int referenceMiddleJunction = 3;
    public static int referenceHighJunction = 4;
    //arm
    public static int referenceArmPickup = 0;
    public static int referenceArmForwards = 1;
    public static int referenceArmBackwards =2;


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
    // Claw
    public static double leftClawGrab = 1.0;
    public static double leftClawRelease = 0;
    public static double rightClawGrab = -1;
    public static double rightClawRelease = 0;

    //claw varaiables
    public static double rightClawClose = .55;
    public static double leftClawClose = .3;
    public static double rightClawPickup = .2;
    public static double rightClawDeliver = .49;

    public static double leftClawOpen = .25;



}