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
            //waiting
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
     * sets all of the drive motors to the same value
     * @param power power in which the drive motors will be set to
     */

    /**
     * pauses the code for a set amount of seconds
     * @param seconds time to wait before resuming code
     */
    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (opModeIsActive() && t.time() <= seconds) {

        }
    }

    protected void drive(double power) {
        setDrivePower(power, power, power, power);
    }

    /**
     * drives the robot in a straight line for a given distance
     * @param power speed which the robot should move at
     * @param inches target for which the robot should try to stop at
     */
    protected void drive(double power, double inches) {
        resetEncoders();
        drive(power);
        double targetPosition = inches * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        double starting = getAngle();
        double right = power;
        double left = power;
        while (opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            double delta = starting - getAngle();
            right = power + (delta / 40);
            left = power - (delta / 40);

            if (Math.abs(right) > 1 || Math.abs(left) > 1) {
                right /= Math.max(right, left);
                left /= Math.max(right, left);
            }
            setDrivePower(right, left, right, left);
        }
        stopDrive();
        telemetry.addLine("Drove " + inches + " inches to target");
        telemetry.update();
    }

    /**
     * moves the robot straight laterally at a given power and distance
     * @param power speed at which it should move
     * @param rotations number of rotations the wheels should spin
     */
    protected void strafeRot(double power, double rotations) {
        resetEncoders();
        waitSec(0.2);
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ORBITAL20_PPR * DRIVE_GEAR_RATIO;

        while (Math.abs((double)rfDrive.getCurrentPosition()) <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }


    protected void driveVector(double target, double direction, double power, double rot, int heading){
        resetEncoders();
        direction = Math.toRadians(direction);
        double adjustment;

        if (rot == 0){
            adjustment = power/2;
        } else {
            adjustment = power;
        }
        if (power < 0){
            adjustment = -adjustment;
        }

        double current = Math.toRadians(getGlobal() % 360);
        target = target * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        drive(direction + current, power, 0);
        while(getRightAbs() < target && getleftAbs() < target && opModeIsActive()){

            //get radian value of the robots angle
            current = Math.toRadians(getGlobal() % 360);
            drive(direction + current, power, 0);

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

        //turn until gyro value is met
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 0) {
            if (heading > degreeTarget) {
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget) {
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
        }
        stopDrive();
        telemetry.addLine("Turned " + degreeTarget + "degrees to target");
        telemetry.update();
        resetAngle();
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
     * scans and returns skystone position
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

    protected void setCollect(double power){
        rColl.setPower(power);
        lColl.setPower(power);
    }
}
