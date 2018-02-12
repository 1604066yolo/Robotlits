package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by dell on 10/9/2017.
 */

@Autonomous

public class AutonRedRelic extends OpMode {

    DcMotor rightWheel;
    DcMotor leftWheel;
    Servo rightClamp1;
    Servo leftClamp2;
    Servo rightClamp2;
    Servo leftClamp1;
    Servo jewel;
    DcMotor RP;
    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    ColorSensor cs;

    public void init() {
        rightWheel = hardwareMap.dcMotor.get("rightMotor");
        leftWheel = hardwareMap.dcMotor.get("leftMotor");
       // rightClamp1 = hardwareMap.servo.get("rightClamp1");
        //leftClamp1 = hardwareMap.servo.get("leftClamp1");
        rightClamp2 = hardwareMap.servo.get("rightClamp2");
        leftClamp2 = hardwareMap.servo.get("leftClamp2");
        jewel = hardwareMap.servo.get("jewel");
        RP = hardwareMap.dcMotor.get("RP");
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cs = hardwareMap.colorSensor.get("cs");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQPBtH3/////AAAAGfidRMTVtEBXniqrShZdwKWHemcE8rF8JDRP51LL4LJLQ892SM/7mcZMzciTMNzmobs919GCa+2ZZYFYxQXYQ0PIHo37gYJBUpgmNMq+4d61X53OWeFgdLkjM+HYoYWzp8CmIXMGGdFPrMVk/CEOFnEAntrnktZe511qJepr3G0OwbeJWhoxEloxI5mqZss7Ysx7tD+zEkJ612MGye3qtsRmgAzwZCBN9xbY+FWffU1WGFARxs3TFPQKGyD4PH5Xc/gGHjxMcViwuHQNdK4wOuPsxiblvX26HpXVH1mc/DVNDM3wG6rS8OHT5JXFbzsalV8WvSKsUfRUijXxkev//CNq8WUbJ6O+IxVt9zbGKqnp";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void start() {
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jewel = hardwareMap.servo.get("jewel");
    }

    public void init_loop() {
        super.init_loop();
    }

    int step = 0;
    int encoderDegsL = 0;
    int encoderDegsR = 0;
    int state;
    boolean isJewelDone;
    public void loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));
            if (vuMark == RelicRecoveryVuMark.LEFT) state = 0;
            else if (vuMark == RelicRecoveryVuMark.CENTER) state = 1;
            else state = 2;
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.addData("state", state);
        //telemetry.addData("left:", leftWheel.getCurrentPosition());//prints on phone
        //telemetry.addData("right:",rightWheel.getCurrentPosition());
        telemetry.addData("step:", step);
        telemetry.update();
        if (step <= 2) {
            pause(1000);
            step++;
        }
        else if (step <= 3){
            //moveForward(4);
            closeClamps();
            step++;
        } else if (step <= 7) {
            if (!isJewelDone) {
                jewel.setPosition(.9);
                pause(1000);
                if (cs.red() > cs.blue() && cs.red() > cs.green()) {
                    turn(-20);
                    pause(1000);
                    jewel.setPosition(0);
                    turn(-3);
                    pause(1000);
                } else {
                    turn(20);
                    pause(750);
                    jewel.setPosition(0);
                    turn(0);
                    pause(750);
                }
                isJewelDone = true;
            }
            step++;
        } else if (step <= 9) {
            jewel.setPosition(0);
            step++;
            pause(1000);
        } else if (step <= 14) {//when step is less than 4 move forward 25 inches and add 1 and pause of
            int a;
            if(step == 0){
                a = 16;
            }
            else if(step == 1){
                a = 19;
            }
            else{
                a = 22;
            }
            telemetry.update();
            moveForward(a);
            step++;
            pause(1000);
        } else if (step == 15) {//when it reaches 5 get current postition
            encoderDegsR = rightWheel.getCurrentPosition();
            encoderDegsL = leftWheel.getCurrentPosition();
            step++;
        } else if (step <= 18) { // when it reaches 6, and 7, it turns right relatively to its position.
            turn(-90);
            step++;
            pause(1000);
        } else if (step == 19) {
            encoderDegsR = rightWheel.getCurrentPosition();
            encoderDegsL = leftWheel.getCurrentPosition();
            step++;
        } else if (step <= 21) {
            openClamps();
            pause(500);
            step++;
        } else if (step <= 23){//when it reaches 13 it opens clamps
            moveForward(10);
            step++;
            pause(1000);
        } else if (step == 24) {
            encoderDegsR = rightWheel.getCurrentPosition();
            encoderDegsL = leftWheel.getCurrentPosition();
            step++;
        } else if (step < 29) {
            moveForward(-5);
            step++;
            pause(1000);
        } else stopMotors();
    }

    public void moveForward(int inches) {
        inches = (int) (inches / (Math.PI * 4) * 1120);
        leftWheel.setTargetPosition(inches + encoderDegsL);
        rightWheel.setTargetPosition(-inches + encoderDegsR);
        leftWheel.setPower(.7);
        rightWheel.setPower(.7);
    }

    public void turn(int degrees) {
        int a = (int) (-degrees * 29);
        leftWheel.setTargetPosition(a + encoderDegsL);
        rightWheel.setTargetPosition(a + encoderDegsR);
        leftWheel.setPower(1);
        rightWheel.setPower(1);
    }

    public void openClamps() {
        //rightClamp1.setPosition(.5);
        //leftClamp1.setPosition(.3);
        rightClamp2.setPosition(.5);
        leftClamp2.setPosition(.3);
    }

    public void closeClamps() {
        //rightClamp1.setPosition(.75);
        //leftClamp1.setPosition(.15);
        rightClamp2.setPosition(.78);
        leftClamp2.setPosition(.12);
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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}