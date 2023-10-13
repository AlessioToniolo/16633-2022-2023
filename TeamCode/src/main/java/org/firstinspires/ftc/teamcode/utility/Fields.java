package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Fields {
    // Use `public static` to show up on DashBoard



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

    /**_____________________SLIDER VARIABLES_______________________**/

    public static double sliderPower = 1;
    public static double sliderOuttake = 0;
    public static double sliderIntake = 0;

    /**_________________________AUTO VISION VARIABLES_______________________**/
    //variables for rectangle to searchIn



    // TODO new servo positions
    public static double airplaneRelease = 0.6;
    public static double airplaneClosed = 0.4;
    public static double v4bIntake = 0;
    public static double v4bDeposit = 0.95;

}