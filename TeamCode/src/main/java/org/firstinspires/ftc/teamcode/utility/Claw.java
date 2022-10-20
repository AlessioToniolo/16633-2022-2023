package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo right;
    public Servo left;

    public Claw(Servo right, Servo left){
        this.right = right;
        this.left = left;
    }

}
