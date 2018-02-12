package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by dell on 2/11/2018.
 */

@TeleOp
public class FlywheelsTest extends OpMode {
    DcMotor rightWheel;
    DcMotor leftWheel;

    public void init() {
        rightWheel = hardwareMap.dcMotor.get("rightMotor");
        leftWheel = hardwareMap.dcMotor.get("leftMotor");


    }

    public void loop() {
        if (gamepad1.right_bumper) {
            rightWheel.setPower(1);
            leftWheel.setPower(-1);
        } else if (gamepad1.left_bumper) {
            rightWheel.setPower(-1);
            leftWheel.setPower(1);
        } else {
            rightWheel.setPower(0);
            leftWheel.setPower(0);
        }
    }
}
