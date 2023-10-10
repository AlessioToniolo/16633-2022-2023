package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Fields {
    // Use `public static` to show up on DashBoard


    //odo variable
    public static double odoRetractDelay = 3.5;
    //the directions which would retract the odo pods
    public static final CRServo.Direction rightOdoDirection = CRServo.Direction.REVERSE;
    public static final CRServo.Direction leftOdoDirection = CRServo.Direction.FORWARD;
    public static final CRServo.Direction middleOdoDirection = CRServo.Direction.REVERSE;



    /**________________MISCELLANOUS ARM AND SLIDER VARIABLES___________________________**/
    //Max and Min Slider
    public static int sliderMinimumTarget = 0;
    public static int sliderMaximumTarget=3000;
    //Max and Min arm
    public static int armMinimumTarget = 0;
    public static int armMaximumTarget = 2000;
    //ARM AND SLIDER SPEED
    public static double armSpeed = .5;
    public static double sliderSpeed = 1;

    public static double highArmSpeed = .425;

    public static int sliderStackUp = 1752;
    public static int armStackUp=100;


    /**________________________________REFERENCE AND ENCODER SLIDER VARIABLES__________________________**/
    public static final int referenceSliderGround =0;
    public static final int referenceSliderConeStack=1;
    public static final int referenceSliderForwardLow = 2;
    public static final int referenceSliderForwardMid = 3;
    public static final int referenceSliderForwardHigh = 4;
    public static final int referenceSliderBackwardsHigh = 5;
    public static final int referenceSliderBackwardsMid = 6;
    public static final int referenceSliderBackwardsLow = 7;



    public static int sliderGround=0;
    public static int sliderConeStack=683;
    public static int sliderForwardLow=0;//updated 1/16
    public static int sliderForwardMid=1400;//updated 1/16
    public static int sliderForwardHigh=2200;//updated 1/31
    public static int sliderBackwardsHigh=2658;//update1/31
    public static int sliderBackMid=1530;//updates 1/17
    public static int sliderBackLow = 0;//updated 1/16
    public static int sliderSuperLow = 200;

    /**_______________________REFERENCE AND ENCODER ARM VARIABLES___________________________**/

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
    public static int armConeStack = 0;
    public static int armForwardLow=657;//updated 1/16
    public static int armForwardMid=657;//updated 1/16
    public static int armForwardHigh = 809;//updated 1/31
    public static  int armBackwardsHigh = 1600;//updated 2/22
    public static final int autoArmBackwardsHigh = 1548;
    public static int armBackwardsMid = 1574;//updated 1/17
    public static int armBackwardsLow = 1484;//updated 1/16

    /**________________________CONE STACK ARM AND SLIDER VARIABLES___________________________________**/
    //Fields for cone stack
    public static int coneStack5 = 683; // updated 1/31
    public static int coneStack4 = 525; // updated 1/31
    public static int coneStack3 = 331;
    public static int coneStack2 = 155;
    public static int coneStack1 = Fields.sliderGround;


    /**___________________BEACON VARIABLES________________________**/
    public static int sliderBeacon = 170;
    public static int armBeacon = 0;



    /**______________________________CLAW VARIABLES_______________________**/
    //If you are looking at the robot from the front then the right claw will be on the left an dhte left claw will be on the right. Just Remember
    //the left and right corrsepond to the robots perspective of front and back
    public static double leftClawClose = .927;
    public static double leftClawPickup = .823;
    public static double leftClawDeliver = .894;
    public static double leftClawCapstone = .875;


    public static double rightClawPickup = .51;
    public static double rightClawDeliver = .399;
    public static double rightClawClose = .356;
    public static double rightClawCapstone = .38;

    /**_________________________AUTO VISION VARIABLES_______________________**/
    //variables for rectangle to searchIn
    public static  int subRectX = 51;
    public static  int subRectY = 87;
    public static  int subRectWidth=96;
    public static  int subRectHeight=114;


    /**_________________________OLD AUTO VARIABLES______________________**/
    public static int LowCycleTurnAngle = 70;
    public static int LowCycleMove = 2;
    public static double highBackAngle = -30;
    public static double highFrontAngle = 49;
    public static double autoConePickup = 52;

    /**______________________New Auto Variables_____________**/
    public static int autoArmForwardHigh=860;

    // TODO new servo positions
    public static double airplaneRelease = 0.6;
    public static double airplaneClosed = 0.4;
    public static double v4bIntake = 0;
    public static double v4bDeposit = 0.95;

}