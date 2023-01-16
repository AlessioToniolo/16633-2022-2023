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
    public static  int subRectHeight=55;

    public static double armSpeed = .5;
    public static double sliderSpeed = 1;

    // Color sensor
    public static double distanceToClose = 2.5;

    //Slider
    public static int sliderMinimumTarget = 0;
    public static int sliderMaximumTarget=1800;

    public static int armMinimumTarget = 0;
    public static int armMaximumTarget = 2000;

    //refrence variables
    //slider
    public static final int referenceSliderGround =0;
    public static final int referenceSliderConeStack=1;
    public static final int referenceSliderForwardLow = 2;
    public static final int referenceSliderForwardMid = 3;
    public static final int referenceSliderForwardHigh = 4;
    public static final int referenceSliderBackwardsHigh = 5;
    public static final int referenceSliderBackwardsMid = 6;
    public static final int referenceSliderBackwardsLow = 7;



    public static int sliderGround=0;
    public static int sliderConeStack=523;
    public static int sliderForwardLow=0;//updated 1/16
    public static int sliderForwardMid=1145;//updated 1/16
    public static int sliderForwardHigh=1574;
    public static int sliderBackwardsHigh=1780;
    public static int sliderBackMid=587;
    public static int sliderBackLow = 0;//updated 1/16
    public static int sliderSuperLow = 200;

    public static final int referenceArmGround = 0;
    public static final int referenceArmConeStack = 1;
    public static final int referenceArmForwardLow = 2;
    public static final int referenceArmForwardMid = 3;
    public static final int referenceArmForwardsHigh = 4;
    public static final int referenceArmBackwardsHigh=5;
    public static final int referenceArmBackwardsMid=6;
    public static final int referenceArmBackwardsLow = 7;

    public static int armGround = 0;
    public static int armAutoGround = 50;
    public static int armConeStack = 95;
    public static int armForwardLow=657;//updated 1/16
    public static int armForwardMid=657;//updated 1/16
    public static int armForwardHigh = 780;
    public static  int armBackwardsHigh = 1496;
    public static int armBackwardsMid = 1481;
    public static int armBackwardsLow = 1484;//updated 1/16

    public static int LowCycleTurnAngle = 70;
    public static int LowCycleMove = 2;
    public static double highBackAngle = -30;
    public static double highFrontAngle = 49;
    public static double autoConePickup = 52;

    //public static int armDepositForwardsMid = 1454;
    //Fields for cone stack
    public static int coneStack5 = 480;//522
    public static int coneStack4 = 320;
    public static int coneStack3 = 230;
    public static int coneStack2 = 85;
    public static int coneStack1 = Fields.sliderGround;
    //set arm to ground too

    public static int sliderBeacon = 170;
    public static int armBeacon = 0;








    //claw varaiables
    // todo oriented as if you were loking directly at
    // the front of the robot for left and right
    public static double leftClawClose = .55;
    public static double leftClawPickup = .45;
    public static double leftClawDeliver = .4;


    public static double rightClawPickup = .67;
    public static double rightClawDeliver = .46;
    public static double rightClawClose = .46;



}