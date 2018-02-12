package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by dell on 10/23/2017.
 */

@TeleOp
public class ColorSensorTest extends OpMode{

    ColorSensor cs;

    public void init() {
        cs = hardwareMap.colorSensor.get("color");
    }

    public void loop() {
        if(cs.red() > cs.blue() && cs.red() > cs.green()) {
            telemetry.addData("seeing red", cs.red());
        } else {
            telemetry.addData("not seeing red", cs.red());
        }
    }

}
