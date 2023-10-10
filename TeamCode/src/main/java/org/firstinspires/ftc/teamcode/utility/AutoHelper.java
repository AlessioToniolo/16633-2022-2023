package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoHelper {
    private BaseRobot robot;
    private ElapsedTime runtime;

    public AutoHelper(BaseRobot robot, ElapsedTime runtime) {
        this.robot = robot;
        this.runtime = runtime;
    }

    // FUNCTIONS
    public void clearConeFromStack() {
        sliderRunTo(Fields.sliderForwardMid);
    }

    public void liftHighGoal(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh);
            armRunTo(Fields.autoArmBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
            armRunTo(Fields.armForwardHigh);
        }
        delay(3);
    }
    public void liftHighGoal(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh, power);
            armRunTo(Fields.autoArmBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh, power);
            armRunTo(Fields.armForwardHigh, power);
        }
        delay(3);
    }
    public void fastLiftHigh(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh);
            armRunTo(Fields.autoArmBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftHigh(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-75, power);
            armRunTo(Fields.autoArmBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-70, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void fastLiftHigher(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh+50);
            armRunTo(Fields.autoArmBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftHigher(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh+50, power);
            armRunTo(Fields.autoArmBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh+50, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void fastLiftLower(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-150);
            armRunTo(Fields.autoArmBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-200);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftLower(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-150, power);
            armRunTo(Fields.autoArmBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-140, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void dunk() {
        sliderRunTo(robot.slider.getCurrentPosition()+200);
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack+60);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStackLess() {
        sliderRunTo(Fields.sliderConeStack-200);
        armRunTo(Fields.armConeStack);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderForwardLow);
    }
    public void liftSuperSlightly() {sliderRunTo(Fields.sliderSuperLow);}
    public void liftHalfway(double power) {
        sliderRunTo(Fields.sliderBackMid, power);
        armRunTo(Fields.armBackwardsMid-300, power);
    }
    public void openClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
        delay(1);
    }
    public void fastOpenClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
    }
    public void actuallyOpenClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
    }
    public void resetLift() {
        armRunTo(Fields.armAutoGround, 0.8);
        sliderRunTo(Fields.sliderGround);
        delay(1.5);
    }
    public void fastResetLift() {
        armRunTo(Fields.armGround);
        sliderRunTo(Fields.sliderGround);
    }
    public void closeClaw() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
        delay(.5);
    }
    public void fastCloseClaw() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
        delay(.5);
    }
    public void lowerChainBar(double power, int difference) {
        armRunTo(Fields.autoArmBackwardsHigh+difference, power);
        //sliderRunTo(Fields.sliderBackwardsHigh+difference);
    }

    // Helper functions
    private void armRunTo(int position){
        armRunTo(position, 1);
    }
    private void armRunTo(int position, double power){
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }
    private void sliderRunTo(int position){
        sliderRunTo(position, 1);
    }
    private void sliderRunTo(int position, double power){
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(power);
        robot.sideSlider.setTargetPosition(position);
        robot.sideSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sideSlider.setPower(power);
    }
    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t/* && !isStopRequested()*/) {
            //dfsdfa
        }
    }
}
