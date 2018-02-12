package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by dell on 10/28/2017.
 */

@Autonomous
public class ScissorAutonTest extends OpMode{

    DcMotor rp;
    DcMotor scissor;

    public void init(){
        rp = hardwareMap.dcMotor.get("rp");
        scissor = hardwareMap.dcMotor.get("scissor");
    }

    public void loop() {

    }


}
