package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by dell on 1/15/2018.
 */
@TeleOp
public class Conveyor extends OpMode {

    DcMotor fly1;
    DcMotor fly2;

    public void init() {
        fly1 = hardwareMap.dcMotor.get("fly1");
        fly2 = hardwareMap.dcMotor.get("fly2");
    }

    public void loop() {
        if (gamepad1.left_bumper) {
            fly1.setPower(1);
        } else fly1.setPower(0);

        if (gamepad1.right_bumper) {
            fly2.setPower(-1);
        } else fly2.setPower(0);
    }

}
