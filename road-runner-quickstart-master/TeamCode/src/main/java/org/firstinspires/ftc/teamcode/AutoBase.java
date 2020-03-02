package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.util.List;


public abstract class AutoBase extends RobotHardware {

    private static final String VUFORIA_KEY =
            "AeZpymf/////AAABmS03EkjpbEZAmqpo0r8H+RFmyvabiay9wA3Vp8xwE8mULQirD/3HvaOjEt8kegr9ER4fBTw5Um3+oT7VcI04wAFdW3VILaGHxhqM8mAyU5QxJnRsOCM8i4ZaRp758aMfY9MwPwEh1TjHR4fnY9LsiQ5WpP7bgWflioTUCP3CiWytU6pS2575KItpWOhRVer2MIky5KsFf9mnZoj69AhT3tz+Jj0+wm48gV1eMOawb7j8STSUG6jWODEHmxiriknKuP/6U4fpfT4wSCMbdl4PwcTG6gHjnRp93RUkCnGZalGg2BGeXXHwJLEowRU4b5wO0SbsBlJtsQ3KlFuJbSWllsjpqvt9qFI4ykn4dSMxueky";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /**
     * Initialize the Vuforia localization engine.
     */
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.65;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    protected SkystonePosition getSkyPos(boolean blue){

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        telemetry.addData("# Object Detected", updatedRecognitions.size());

        int skyX = -1;
        int stone1X = -1;
        int stone2X = -1;

        for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel() == LABEL_SECOND_ELEMENT){
                skyX = (int) recognition.getLeft();
            } else if (stone1X == -1){
                stone1X = (int) recognition.getLeft();
            } else {
                stone2X = (int) recognition.getLeft();
            }
        }

        tfod.deactivate();

        if (blue){
            if (skyX == -1){
                telemetry.addData("Skystone: ", "Right");
                telemetry.update();
                return SkystonePosition.RIGHT;
            } else if (skyX > stone1X){
                telemetry.addData("Skystone: ", "Center");
                telemetry.update();
                return SkystonePosition.CENTER;
            } else {
                telemetry.addData("Skystone: ", "Left");
                telemetry.update();
                return SkystonePosition.LEFT;
            }
        } else {
            if (skyX == -1){
                telemetry.addData("Skystone: ", "Left");
                telemetry.update();
                return SkystonePosition.LEFT;
            } else if (skyX > stone1X){
                telemetry.addData("Skystone: ", "Right");
                telemetry.update();
                return SkystonePosition.RIGHT;
            } else {
                telemetry.addData("Skystone: ", "Center");
                telemetry.update();
                return SkystonePosition.CENTER;
            }
        }
    }

    /**
     * resets the drive motor encoders
     */

    protected void resetEncoders() {
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && rfDrive.getCurrentPosition() > 3 && lfDrive.getCurrentPosition() > 3) {
        }
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && lfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            //waiting for changes to finish
        }
    }

    /**
     * gets the absolute value of the right drive motor encoder positions
     * @return int - absolute value of right front wheel
     */
    protected int getRightAbs() {
        return Math.abs(rfDrive.getCurrentPosition());
    }

    /**
     * gets the absolute value of the left drive motor encoder positions
     * @return int - absolute value of left front wheel
     */
    protected int getleftAbs() {
        return Math.abs(lfDrive.getCurrentPosition());
    }

    /**
     * pauses the code for a set amount of seconds
     * @param seconds time to wait before resuming code
     */
    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (!isStopRequested() && t.time() <= seconds) { }
    }

    /**
     * sets all of the drive motors to the same value
     * @param power power in which the drive motors will be set to
     */
    protected void drive(double power) {
        setDrivePower(power, power, power, power);
    }

    protected void driveVector(double target, double direction, double power, double rot, int heading, boolean decelerate){
        resetEncoders();
        direction = Math.toRadians(direction);
        double current = Math.toRadians(getGlobal() % 360);
        target = target * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;

        drive(direction + current, power, 0);
        while(getRightAbs() < target && getleftAbs() < target && opModeIsActive()){

            //get radian value of the robots angle
            current = Math.toRadians(getGlobal() % 360);

            if (Math.abs(getGlobal() - heading) > 35){

                if (getGlobal() > heading) {
                    rot = power;
                } else if (getGlobal() < heading) {
                    rot = -power;
                } else {
                    rot = 0;
                }

            } else {

                double degreesLeft = Math.abs(getGlobal() - heading);
                if (getGlobal() - heading > 2) {
                    rot = ((degreesLeft/35)*(rot - 0.1) + 0.1);
                } else if (getGlobal() - heading < 2) {
                    rot = -((degreesLeft/35)*(rot - 0.1) + 0.1);
                } else {
                    rot = 0;
                }
                telemetry.addData("degrees left ", degreesLeft);

            }

            double ticksLeft = Math.abs(target - (Math.max(getRightAbs(), getleftAbs())));

            if (ticksLeft > ORBITAL20_PPR * 1.5){
                //drive at full power
            } else if (decelerate){
                //decelerate for last 18 inches
                //power = (ticksLeft/(ORBITAL20_PPR * 1.5)) * (power - (0.15 + Math.abs(Math.cos(direction + current)))) + (0.15 + Math.abs(Math.cos(direction + current)));
                power = (ticksLeft/(ORBITAL20_PPR * 1.5)) * (power - 0.2) + (0.2);

            }


            drive(direction + current, power, rot);
            telemetry.addData("currentAngle", getGlobal());
            telemetry.addData("rot", rot);
            telemetry.update();

        }
        stopDrive();
    }

    protected void driveVectorRot(double target, double direction, double power, double rot, int heading, boolean decelerate){
        resetEncoders();
        direction = Math.toRadians(direction);
        double current = Math.toRadians(getGlobal() % 360);
        target = target * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;

        drive(direction + current, power, 0);
        while(getRightAbs() < target && getleftAbs() < target && opModeIsActive()){

            //get radian value of the robots angle
            current = Math.toRadians(getGlobal() % 360);

            double ticksLeft = Math.abs(target - (Math.max(getRightAbs(), getleftAbs())));

            if (ticksLeft > ORBITAL20_PPR * 1.5){
                //drive at full power
            } else if (decelerate){
                //decelerate for last 18 inches
                //power = (ticksLeft/(ORBITAL20_PPR * 1.5)) * (power - (0.15 + Math.abs(Math.cos(direction + current)))) + (0.15 + Math.abs(Math.cos(direction + current)));
                power = (ticksLeft/(ORBITAL20_PPR * 1.5)) * (power - 0.2) + (0.2);

            }


            drive(direction + current, power, rot);
            telemetry.addData("currentAngle", getGlobal());
            telemetry.addData("rot", rot);
            telemetry.update();

        }
        stopDrive();
    }

    /**
     * turns the robot at a given speed to target angle
     * @param power speed at which to turn
     * @param degreeTarget target angle which it will try to turn to
     */
    protected void turnHeading(double power, int degreeTarget) {
        heading = getAngle();

        //turn at full power until within 30 degrees of target
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 30) {

            if (heading > degreeTarget) {
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget) {
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
            double angle = getGlobal();

            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }

        //turn while scaling down the power as it approaches the target
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 1) {


            double degreesLeft = Math.abs(heading - degreeTarget);
            double newPower = (degreesLeft/30)*(power - 0.1) + 0.1;

            if (heading > degreeTarget) {
                setDrivePower(-newPower, newPower, -newPower, newPower);
            }
            if (heading < degreeTarget) {
                setDrivePower(newPower, -newPower, newPower, -newPower);
            }
            heading = getAngle();

            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }

        stopDrive();
        waitSec(0.4);
        //turn while scaling down the power as it approaches the target
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 1) {


            double degreesLeft = Math.abs(heading - degreeTarget);
            double newPower = (degreesLeft/30)*(power - 0.1) + 0.1;

            if (heading > degreeTarget) {
                setDrivePower(-newPower, newPower, -newPower, newPower);
            }
            if (heading < degreeTarget) {
                setDrivePower(newPower, -newPower, newPower, -newPower);
            }
            heading = getAngle();
            double angle = getGlobal();

            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }

        stopDrive();

        telemetry.addLine("Turned to " + degreeTarget + " degrees");
        telemetry.update();

    }

    protected void turnFoundation(double power, int degreeTarget) {
        heading = getAngle();

        //turn at full power until within 30 degrees of target
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 1) {

            if (heading > degreeTarget) {
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget) {
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
            double angle = getGlobal();

            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }

        stopDrive();

        telemetry.addLine("Turned to " + degreeTarget + " degrees");
        telemetry.update();

    }

    protected void turnGlobal(double power, double degreeTarget) {
        heading = (int)getGlobal();

        //turn at full power until within 30 degrees of target
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 1) {

            if (heading > degreeTarget) {
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget) {
                setDrivePower(power, -power, power, -power);
            }

            heading = (int)getGlobal();

            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }

        stopDrive();

        telemetry.addLine("Turned to " + degreeTarget + " degrees");
        telemetry.update();

    }

    /**
     * resets the angle of the gyroscope method
     */
    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * gets the angle of the gyro
     * @return int - angular heading of the robot
     */
    private int getAngle() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }


    /**
     * turns on both wheels of the collector at a set power
     * @param power speed to turn the wheels at
     */
    protected void setCollect(double power){
        rColl.setPower(power);
        lColl.setPower(power);
    }

    /**
     * sets the foundation hooks to open or closed
     * @param grab
     */
    protected  void setHooks(boolean grab){
        if (grab){
            hookL.setPosition(0);
            hookR.setPosition(1);
        } else {
            hookL.setPosition(0.3);
            hookR.setPosition(0.85);
        }
    }

    /**
     * lowers the stone arm, grabs the stone, and raises it
     */
    protected void grab(){
        stoneGrab.setPosition(0);
        stoneArm.setPosition(0.15);
        waitSec(0.5);
        stoneGrab.setPosition(1);
        waitSec(0.55);
        stoneArm.setPosition(0.6);
        waitSec(0.4);
    }

    /**
     * lowers the stone arm and drops the stone
     */
    protected void drop(){
        stoneArm.setPosition(0.1);
        waitSec(0.2);
        stoneGrab.setPosition(0);
        waitSec(0.2);
        stoneArm.setPosition(0.6);
        waitSec(0.3);
    }
}

