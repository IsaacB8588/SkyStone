package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class RobotHardware extends RobotBase {

    //declare hardware
    protected DcMotor rfDrive;
    protected DcMotor rbDrive;
    protected DcMotor lfDrive;
    protected DcMotor lbDrive;

    protected DcMotor lColl;
    protected DcMotor rColl;

    protected DcMotor spool;

    protected Servo grab;
    protected Servo flip;

    protected Servo noCap;

    protected Servo hookL;
    protected Servo hookR;

    protected Servo stoneArm;
    protected Servo stoneGrab;

    protected BNO055IMU imu;

    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;

    protected double globalAngle;

    protected int heading;

    //final variables for moving robot to distance
    protected final double WHEEL_DIAMTER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMTER*Math.PI;
    protected final double ORBITAL20_PPR = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;

    protected double FORWARD_RATIO = 1;
    protected double STRAFE_RATIO = 1;
    protected double TURN_RATIO = 0.7;

    int toggleCount = 0;
    boolean toggleSpeed = false;

    private boolean escapeClause = false;

    @Override
    public void initRobot (RobotRunType robotRunType){

        rfDrive = hardwareMap.dcMotor.get("rf");
        rbDrive = hardwareMap.dcMotor.get("rb");
        lfDrive = hardwareMap.dcMotor.get("lf");
        lbDrive = hardwareMap.dcMotor.get("lb");

        rfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rColl = hardwareMap.dcMotor.get("rc");
        lColl = hardwareMap.dcMotor.get("lc");

        rColl.setDirection(DcMotorSimple.Direction.REVERSE);

        spool = hardwareMap.dcMotor.get("elv");

        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grab = hardwareMap.servo.get("grab");
        grab.setPosition(0.3);

        flip = hardwareMap.servo.get("flip");
        flip.setPosition(0);

        noCap= hardwareMap.servo.get("noCap");
        noCap.setPosition(1);

        hookL = hardwareMap.servo.get("hl");
        hookR = hardwareMap.servo.get("hr");
        hookL.setPosition(0);
        hookR.setPosition(1);

        stoneArm = hardwareMap.servo.get("stone");
        stoneGrab = hardwareMap.servo.get("stoneg");

        //initialize gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //post to telemetry when gyro is calibrating
        telemetry.addData("Mode", "Calibrating");
        telemetry.update();

        if (robotRunType == RobotRunType.AUTONOMOUS){
            stoneArm.setPosition(1);
            stoneGrab.setPosition(1);
        }

        //post to telemetry when gyro is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated() && !escapeClause){
            sleep(50);
            idle();

            if (gamepad1.right_bumper && gamepad1.left_bumper){
                escapeClause = true;
            }
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
        telemetry.update();

    }

    protected static class Wheels {
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }

    /**
     * method used to drive a holonomic drive train given a vector for direction and power
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     * @return Wheels - creates new Wheels class
     */
    protected Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s = Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }

    /**
     * method to scale values to within a certain range
     *
     * @param xs values to be used to create the scale
     * @return double - scale to be used by the upper shell
     */
    protected static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }

        return ret;
    }

    /**
     * Uses the wheels classes to apply the vector driven holonomic formula to the four wheels of a
     * holonomic drivetrain
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     */
    protected void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lfDrive.setPower(w.lf);
        rfDrive.setPower(w.rf);
        lbDrive.setPower(w.lr);
        rbDrive.setPower(w.rr);
    }

    /**
     * method which allows the rev integrated gyroscope to be used without the wrap around values
     * where it resets at 180 degrees
     *
     * @return double - unrestricted gyroscope value of the robot
     */
    public double getGlobal(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * method to drive the robot using field-centric drive
     *
     * movement - left joystick
     * rotation - right joystick x-axis
     * slow-mode - right bumper
     */
    protected void FieldCentricDrive(){


        if (gamepad1.y){
            toggleCount++;
        } else {
            toggleCount = 0;
        }

        if (toggleCount == 1){
            toggleSpeed = !toggleSpeed;
            if (!toggleSpeed){
                TURN_RATIO = 0.7;
                FORWARD_RATIO = 1;
                STRAFE_RATIO = 1;
            } else {
                TURN_RATIO = 0.3;
                FORWARD_RATIO = 0.5;
                STRAFE_RATIO = 0.5;
            }
        }


        //pull coordinate values from x and y axis of joystick
        double x1 = gamepad1.left_stick_x * STRAFE_RATIO, y1 = -gamepad1.left_stick_y * FORWARD_RATIO;
        //create polar vector of given the joystick values
        double v = Math.sqrt(x1 * x1 + y1 * y1);
        double theta = Math.atan2(x1, y1);
        //get radian value of the robots angle
        double current = Math.toRadians(getGlobal() % 360);
        //apply values to vector-based holonomic drive
        drive(theta + current, v, gamepad1.right_stick_x * TURN_RATIO);
    }

    /**
     * stops all of the drive wheels of the robot
     */
    protected void stopDrive() {
        setDrivePower(0, 0, 0 ,0);
    }

    /**
     * Method to apply power to all four of the motors with a single line of code
     *
     * @param rightFrontPower power of the right front wheel
     * @param leftFrontPower power of the left front wheel
     * @param rightBackPower power of the right back wheel
     * @param leftBackPower power of the left back wheel
     */
    protected void setDrivePower (double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {
        rfDrive.setPower(rightFrontPower);
        lfDrive.setPower(leftFrontPower);
        rbDrive.setPower(rightBackPower);
        lbDrive.setPower(leftBackPower);

    }
}


