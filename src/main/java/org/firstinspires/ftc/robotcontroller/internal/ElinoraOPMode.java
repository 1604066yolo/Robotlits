package org.firstinspires.ftc.robotcontroller.internal;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dell on 10/6/2017.
 */

public class ElinoraOPMode extends OpMode {

    DcMotor rightWheel;
    DcMotor leftWheel;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor RP1;
    DcMotor RP2;
    DcMotor flyWheel1;
    DcMotor flyWheel2;

    Servo s1;
    Servo s2;


    public void init() {
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        leftWheel = hardwareMap.dcMotor.get("leftWheel");

      /*  frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");  */
        RP1 = hardwareMap.dcMotor.get("RP1");
        RP2 = hardwareMap.dcMotor.get("RP2");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
    }

    int div = 1;

    public void loop() {

        /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4); */

        //rightWheel.setPower(gamepad1.right_stick_y/div);
        //leftWheel.setPower(-gamepad1.left_stick_y/div);

        if(gamepad1.b){
            if(div == 1){
                div = 2;
                pause(200);
            }
            else{
                div = 1;
                pause(200);
            }
        }

        /*if (gamepad2.left_bumper) {
            flyWheel1.setPower(1);
            flyWheel2.setPower(1);
        } else if (gamepad2.right_bumper) {
            flyWheel1.setPower(-1);
            flyWheel2.setPower(-1);
        }*/

        RP1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        RP2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        if(gamepad1.x) {
            s1.setPosition(.7);
            s2.setPosition(.7);
        }
        if(gamepad1.y){
            s1.setPosition(0);
            s2.setPosition(0);
        }
    }

    public void pause(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }

}

