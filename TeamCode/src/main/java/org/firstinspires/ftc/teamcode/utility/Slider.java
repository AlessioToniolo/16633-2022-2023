package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Slider {
    public static final int CONEPICKUP = 0;
    public static final int STACKPICKUP = 1;
    public static final int GROUND = 2;
    public static final int LOW = 3;
    public static final int MIDDLE = 4;
    public static final int HIGH = 5;




    private DcMotor right;
    private DcMotor left;
    private Servo rightServo;
    private Servo leftServo;
    private boolean loopActive = false;
    private int targetPosition = 0;

    /**
     * Dual Slider is a class which holds two motors which correspond to the left and right motors of a slider
     * Its provides helpful methods for using these sliders.
     * @param right
     * @param left
     */
    public Slider(DcMotor right, DcMotor left, Servo rightServo, Servo leftServo){
        this.right = right;
        this.left = left;
        this.rightServo=rightServo;
        this.leftServo=leftServo;
    }


    /**public void goToPos(int pos){
        if(pos == Slider.CONEPICKUP){
            armPickup();
            setTargetPosition(Fields.conePickUp);
        }
        else if(pos == Slider.STACKPICKUP){
            armPickup();
            setTargetPosition(Fields.stackPickUp);
        }
        else if(pos == Slider.GROUND){
            setTargetPosition(Fields.ground);
            armPickup();
        }
        else if(pos == Slider.LOW){
            setTargetPosition(Fields.low);
            armDeliver();
        }
        else if(pos == Slider.MIDDLE){
            setTargetPosition(Fields.middle);
            armDeliver();
        }
        else if(pos == Slider.HIGH){
            setTargetPosition(Fields.high);
            armDeliver();
        }
        if(!loopActive){
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setPower(1);
        }

    }**/

    /**public void armDeliver(){
        rightServo.setPosition(Fields.deliver);
        leftServo.setPosition(Fields.deliver);
    }
    public void armPickup(){
        rightServo.setPosition(Fields.rest);
        leftServo.setPosition(Fields.rest);
    }**/
    /*

    public void setServoPower(double power){
        rightServo.setPower(power);
        leftServo.setPower(power);
    }

     */



    /**
     * RunTo-given a target position, this method will run the motors to that target position
     * @param position- the target positon to runTO
     */
    public void runTo(int position){
        loopActive = false;
        right.setTargetPosition(position);
        left.setTargetPosition(position);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(1);

    }

    /**
     * RunAsync: when called will start a loop which runs in a seperate thread that keeps the slider at
     * its target position. setTargetPosition can be called to increases the target and stopAsync is called to stop the loop
     * @return void
     */
    public void runAsync(){
        if(!loopActive){//only start the slider loop if it isnt already running
        Thread sliderThread = new Thread(this::sliderLoop);//create a new thread which runs the sliderLoop method
        sliderThread.start();}//start the new Thread

    }

    /**
     * StopAsync: will stop the loop which runs the lisder to and keeps it at the target position
     */
    public void stopAsync(){
        loopActive=false;
    }

    /**
     * SliderLoop: runs a loop which continuosly sets the motors' target position to the current target positon
     * and sets them at power one.
     * When loopActive becomes false the loop will end and turn the motors off.
     */
    private void sliderLoop(){
        loopActive=true;
        setTargetPosition(targetPosition);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(loopActive){
            setPower(1);
        }
        setPower(0);
    }

    //methods to that do normal motor stuff

    public void setTargetPosition(int position) {
        if(position>Fields.sliderMaximumTarget)position=Fields.sliderMaximumTarget;
        else if(position< Fields.sliderMinimumTarget)position=Fields.sliderMinimumTarget;
        targetPosition = position;
        right.setTargetPosition(targetPosition);
        left.setTargetPosition(targetPosition);
    }
    public int getTargetPosition() {
        return targetPosition;
    }
    public int getCurrentPosition() {
        return right.getCurrentPosition();
    }
    public void setMode(DcMotor.RunMode mode) {
        right.setMode(mode);
        left.setMode(mode);
    }
    public void setPower(double power) {
        right.setPower(power);
        left.setPower(power);
    }
    public double getPower() {
        return right.getPower();
    }


}
