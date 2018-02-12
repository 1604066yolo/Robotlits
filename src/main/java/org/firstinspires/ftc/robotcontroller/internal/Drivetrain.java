package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by dell on 11/27/2017.
 */

public class Drivetrain {

    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo rightClamp1;
    Servo leftClamp2;
    Servo rightClamp2;
    Servo leftClamp1;
    DcMotor RP;

    int encoderDegsL;
    int encoderDegsR;

    public Drivetrain(HardwareMap hardwareMap, DcMotor leftWheel, DcMotor rightWheel) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        rightClamp1 = hardwareMap.servo.get("rightClamp1");
        leftClamp1 = hardwareMap.servo.get("leftClamp1");
        rightClamp2 = hardwareMap.servo.get("rightClamp2");
        leftClamp2 = hardwareMap.servo.get("leftClamp2");
        RP = hardwareMap.dcMotor.get("RP");
        RP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateEncoders() {
        encoderDegsL += leftWheel.getCurrentPosition();
        encoderDegsR += rightWheel.getCurrentPosition();
    }

    public void moveForward(int inches, DcMotor leftWheel, DcMotor rightWheel) {
        inches = (int) (inches / (Math.PI * 4) * 1120);
        leftWheel.setTargetPosition(inches + encoderDegsL);
        rightWheel.setTargetPosition(-inches - encoderDegsR);
        leftWheel.setPower(1);
        rightWheel.setPower(1);
        //updateEncoders();
    }

    public void turnLeft(){
        leftWheel.setTargetPosition(-1700 + encoderDegsL);
        rightWheel.setTargetPosition(-1700 + encoderDegsR);
        leftWheel.setPower(.4);
        rightWheel.setPower(.4);
        updateEncoders();
    }

    public void turnRight() {
        leftWheel.setTargetPosition(1800 + encoderDegsL);
        rightWheel.setTargetPosition(1800 + encoderDegsR);
        leftWheel.setPower(1);
        rightWheel.setPower(1);
        updateEncoders();
    }

    public void openClamps() {
        rightClamp1.setPosition(.4);
        leftClamp1.setPosition(.6);
        rightClamp2.setPosition(.4);
        leftClamp2.setPosition(.6);
    }

    public void closeClamps() {
        rightClamp1.setPosition(.8);
        leftClamp1.setPosition(.1);
        rightClamp2.setPosition(.8);
        leftClamp2.setPosition(.1);
    }

    public void stopMotors() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void pause(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }

    public void setRPPower(float power) {
        RP.setPower(power);
    }

}
