package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;


public abstract class AutoBase extends RobotHardware {

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
        while (opModeIsActive() && t.time() <= seconds) { }
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
            double angle = getGlobal();

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

    protected boolean isSkystone(ColorSensor side){

        if (side.red()/side.blue() > 1.6){
            return false;
        } else {
            return true;
        }
    }

    /**
     * scans and returns skystone position using color sensors
     */
    protected SkystonePosition skystonePosition (){

        if (isSkystone(colorL) && !isSkystone(colorR)){
            return SkystonePosition.LEFT;
        } else if (!isSkystone(colorL) && !isSkystone(colorR)){
            return SkystonePosition.RIGHT;
        } else if (!isSkystone(colorL) && isSkystone(colorR)){
            return SkystonePosition.CENTER;
        } else {
            return SkystonePosition.RIGHT;
        }
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
            hookL.setPosition(1);
            hookR.setPosition(0);
        } else {
            hookL.setPosition(0);
            hookR.setPosition(1);
        }
    }

    /**
     * moves the robot in a arc of a set radius
     */
    protected void turnArc(double radius, double power, double angle){

        int targetR = (int) ((ORBITAL20_PPR/WHEEL_CIRC) * ((radius + 7) * 2 * Math.PI * (angle / 360)));
        int targetL = (int) ((ORBITAL20_PPR/WHEEL_CIRC) * ((radius - 7) * 2 * Math.PI * (angle / 360)));


        do{
            setDrivePower(((radius + 7) / Math.max((radius + 7), (radius - 7)) * power), ((radius - 7) / Math.max((radius + 7), (radius - 7)) * power), ((radius + 7) / Math.max((radius + 7), (radius - 7)) * power), ((radius - 7) / Math.max((radius + 7), (radius - 7)) * power));

        }while(Math.abs(getAngle()) < Math.abs(angle) && opModeIsActive());

        setDrivePower(0,0,0,0 );
    }
}
