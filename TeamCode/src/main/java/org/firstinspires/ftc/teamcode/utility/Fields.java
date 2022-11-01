package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
public class Fields {
    // Use `public static`

    //Auto
    public static  int subRectX = 0;
    public static  int subRectY = 0;
    public static  int subRectWidth=170;
    public static  int subRectHeight=72;

    public static double motorSpeed = 1;



    //Slider
    public static int sliderMinimumTarget = 0;
    public static int sliderMaximumTarget=1600;

    public static int armMinimumTarget = 0;
    public static int armMaximumTarget = 2000;

    public static int armDepositForward=546;
    public static int armDepositBackwards=1761;
    public static final int armDepostBackwardsHigh = 1551;
    public static final int armDepostForwardsHigh = 830;

    public static int armPickup = 110;

    //refrence variables
    //slider
    public static int referenceGroundPickup =0;
    public static int referenceConeStackPickup=1;
    public static int referenceLowJunction = 2;
    public static int referenceMiddleJunction = 3;
    public static int referenceHighJunction = 4;
    //arm
    public static final int referenceArmPickup = 0;
    public static final int referenceArmForwards = 1;
    public static final int referenceArmForwardsHigh =2;
    public static final int referenceArmBackwardsHigh = 3;
    public static final int referenceArmBackwards =4;



    /**
    public static int armIntakeLevel=0;
    public static int armGroundJunctionLevel=0;
    public static int armLowJunctionLevel=540;
    public static int armMidJunctionLevel=0;
    public static int armHighJunctionLevel=0;**/

    public static int sliderGroundPickup=110;
    public static int sliderConeStackPickup=400;
    public static int sliderLowJunctionLevel=690;
    public static int sliderMidJunctionLevel=1200;
    public static int sliderHighJunctionLevel=1450;


    //claw varaiables
    public static double leftClawClose = .3;
    public static double leftClawPickup = .25;
    public static double leftClawDeliver = .1;


    public static double rightClawPickup = .2;
    public static double rightClawDeliver = .45;
    public static double rightClawClose = .55;



}