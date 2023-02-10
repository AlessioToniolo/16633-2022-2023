package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;

public class AutoFunctions {
    private BaseRobot robot = new BaseRobot();

    public AutoFunctions(HardwareMap hwMap) {
        robot.init(hwMap);
    }

    // TODO lift functions
    public void liftFrontHigh() {
        sliderRunTo(Fields.sliderForwardHigh);
        armRunTo(Fields.armForwardHigh);
    }

    public void liftFrontHigh(double power) {
        sliderRunTo(Fields.sliderForwardHigh, power);
        armRunTo(Fields.armForwardHigh, power);
    }

    public void liftFrontHigh(double sliderPower, double armPower) {
        sliderRunTo(Fields.sliderForwardHigh, sliderPower);
        armRunTo(Fields.autoArmForwardHigh, armPower);
    }

    public void liftBackHigh() {
        sliderRunTo(Fields.sliderBackwardsHigh);
        armRunTo(Fields.armBackwardsHigh);
    }

    public void liftBackHigh(double power) {
        sliderRunTo(Fields.sliderBackwardsHigh, power);
        armRunTo(Fields.armBackwardsHigh, power);
    }

    public void liftBackHigh(double sliderPower, double armPower) {
        sliderRunTo(Fields.sliderBackwardsHigh, sliderPower);
        armRunTo(Fields.armBackwardsHigh, armPower);
    }



    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack);
        armRunTo(Fields.armConeStack);
    }

    public void liftConeStack(double power) {
        sliderRunTo(Fields.sliderConeStack, power);
        armRunTo(Fields.armConeStack, power);
    }

    public void liftConeStack4() {
        sliderRunTo(Fields.coneStack4);
        armRunTo(Fields.armConeStack);
    }

    public void liftConeStack4(double power) {
        sliderRunTo(Fields.coneStack4, power);
        armRunTo(Fields.armConeStack, power);
    }

    public void liftConeStack3() {
        sliderRunTo(Fields.coneStack3);
        armRunTo(Fields.armConeStack);
    }

    public void liftConeStack3(double power) {
        sliderRunTo(Fields.coneStack3, power);
        armRunTo(Fields.armConeStack, power);
    }

    public void liftConeStack2() {
        sliderRunTo(Fields.coneStack2);
        armRunTo(Fields.armConeStack);
    }

    public void liftConeStack2(double power) {
        sliderRunTo(Fields.coneStack2, power);
        armRunTo(Fields.armConeStack, power);
    }

    public void hoverForward(double power) {
        //sliderRunTo(Fields.sliderForwardLow);
        armRunTo(Fields.armForwardLow, power);
    }

    public void lowerArmFrontSlightlyFromHigh(int difference) {
        armRunTo(Fields.armForwardHigh - difference);
    }

    public void lowerArmBackSlightlyFromHigh(int difference) {
        armRunTo(Fields.armBackwardsHigh - difference);
    }

    // TODO claw functions
    public void clawClose() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
    }

    public void clawOpen() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
    }

    public void clawDeliver() {
        robot.rightClaw.setPosition(Fields.rightClawDeliver);
        robot.leftClaw.setPosition(Fields.leftClawDeliver);
    }

    // TODO Helper functions
    private void armRunTo(int position) {
        armRunTo(position, 1);
    }

    private void armRunTo(int position, double power) {
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }

    private void sliderRunTo(int position) {
        sliderRunTo(position, 1);
    }

    private void sliderRunTo(int position, double power) {
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(power);
        robot.sideSlider.setTargetPosition(position);
        robot.sideSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sideSlider.setPower(power);
    }

    public void resetAll() {
        sliderRunTo(Fields.sliderGround);
        armRunTo(Fields.armGround);
    }
}
