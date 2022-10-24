package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
public class Fields {
    // Use `public static`

    //Slider
    public static int sliderMinimumTarget = 0;
    public static int sliderMaximumTarget=5000;
    public static int armMinimumTarget = -200;
    public static int armMaximumTarget = -0;
    public static int armIntakeLevel=0;
    public static int armGroundJunctionLevel=0;
    public static int armLowJunctionLevel=0;
    public static int armMidJunctionLevel=0;
    public static int armHighJunctionLevel=0;
    public static int sliderIntakeLevel=0;
    public static int sliderGroundJunctionLevel=0;
    public static int sliderLowJunctionLevel=0;
    public static int sliderMidJunctionLevel=0;
    public static int sliderHighJunctionLevel=0;
    public static int ground;
    public static int low=290;//-300
    public static int middle = 1500;
    public static int high=3000;
    public static int stackPickUp=0;
    public static int conePickUp=0;

    public static double deliver = 70;
    public static double rest = 0;

    // Servo
    /*
    public static double servoDeposit = 0.3;
    public static double servoResting = 0.9;
     */
}