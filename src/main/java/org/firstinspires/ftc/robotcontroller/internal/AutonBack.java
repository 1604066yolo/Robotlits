package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class AutonBack extends OpMode {

    DcMotor rightWheel;
    DcMotor leftWheel;
    Servo rightClamp1;
    Servo leftClamp2;
    Servo rightClamp2;
    Servo leftClamp1;
    DcMotor RP;
    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public void init() {
        rightWheel = hardwareMap.dcMotor.get("rightMotor");
        leftWheel = hardwareMap.dcMotor.get("leftMotor");
        rightClamp1 = hardwareMap.servo.get("rightClamp1");
        leftClamp1 = hardwareMap.servo.get("leftClamp1");
        rightClamp2 = hardwareMap.servo.get("rightClamp2");
        leftClamp2 = hardwareMap.servo.get("leftClamp2");
        RP = hardwareMap.dcMotor.get("RP");
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQPBtH3/////AAAAGfidRMTVtEBXniqrShZdwKWHemcE8rF8JDRP51LL4LJLQ892SM/7mcZMzciTMNzmobs919GCa+2ZZYFYxQXYQ0PIHo37gYJBUpgmNMq+4d61X53OWeFgdLkjM+HYoYWzp8CmIXMGGdFPrMVk/CEOFnEAntrnktZe511qJepr3G0OwbeJWhoxEloxI5mqZss7Ysx7tD+zEkJ612MGye3qtsRmgAzwZCBN9xbY+FWffU1WGFARxs3TFPQKGyD4PH5Xc/gGHjxMcViwuHQNdK4wOuPsxiblvX26HpXVH1mc/DVNDM3wG6rS8OHT5JXFbzsalV8WvSKsUfRUijXxkev//CNq8WUbJ6O+IxVt9zbGKqnp";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void start() {
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void init_loop() {
        super.init_loop();
        closeClamps();
    }

    int step = 0;
    int encoderDegsL = 0;
    int encoderDegsR = 0;

    public void loop() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }
        } else {
            telemetry.addData("VuMark", "not visible");
        }


        telemetry.update();
        telemetry.addData("left:", leftWheel.getCurrentPosition());//prints on phone
        telemetry.addData("right:",rightWheel.getCurrentPosition());
        telemetry.addData("step:", step);
        if (step <= 4) {//when step is less than 4 move forward 25 inches and add 1 and pause of 800
            moveForward(22);
            pause(1000);
            step++;
        }  else if (step <= 5) {
            openClamps();
            pause(1000);
            step++;
        } else {
            stopMotors();
        }
    }

    public void moveForward(int inches) {
        inches = (int) (inches / (Math.PI * 4) * 1120);
        leftWheel.setTargetPosition((int) inches);
        rightWheel.setTargetPosition((int) -inches);
        leftWheel.setPower(1);
        rightWheel.setPower(1);
    }


    public void moveBackward(int inches) {
        double inch = inches / (Math.PI * 4) * 1120;
        leftWheel.setTargetPosition((int) -inch);
        rightWheel.setTargetPosition((int) inch);
        leftWheel.setPower(.4);
        rightWheel.setPower(.4);
    }

    public void turnLeft() {
        leftWheel.setTargetPosition(-2150 + encoderDegsL);
        rightWheel.setTargetPosition(-2150 + encoderDegsR);
        leftWheel.setPower(.4);
        rightWheel.setPower(.4);
    }

    public void turnRight() {
        leftWheel.setTargetPosition(1800 + encoderDegsL);
        rightWheel.setTargetPosition(1800 + encoderDegsR);
        leftWheel.setPower(1);
        rightWheel.setPower(1);
    }

    public void openClamps() {
        rightClamp1.setPosition(.4);
        leftClamp1.setPosition(.6);
        rightClamp2.setPosition(.4);
        leftClamp2.setPosition(.6);
    }

    public void closeClamps() {
        rightClamp1.setPosition(.7);
        leftClamp1.setPosition(.2);
        rightClamp2.setPosition(.7);
        leftClamp2.setPosition(.2);
    }

    public void stopMotors() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void scanImage(int degrees) {

    }

    public void scanJewel(int degrees) {

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