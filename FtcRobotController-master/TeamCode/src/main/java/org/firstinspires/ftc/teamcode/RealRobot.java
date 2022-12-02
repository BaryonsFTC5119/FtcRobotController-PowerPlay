package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class RealRobot {

    static final double     COUNTS_PER_MOTOR_REV    = 145.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference || Previous value of 3.93701
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public final DcMotor lf, lr, rf, rr;

    public final DcMotor ltrolley;
    //public final DcMotor ltrolley;

    //, carousel;

    //public final Servo lchain, rchain, lclaw, rclaw;
    //public final Servo lchain;
    //public final Servo lclaw;




    //public final Servo grabber,track, trayL, trayR;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;
    private int armLevel0 = 0;
    private int armLevel1 = 0;
    private int armLevel2 = 0;
    private int zeroPosition = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true  ;
    private static final String VUFORIA_KEY =
            "AQrfl2X/////AAABmXgYPrwW30wdoBAHztdXSDgfzAtx/3aneWzwyCjSRj16HSy36qQ27HjnVGpjTF+XeOKXM5S3kX+HHUnC8HDJBAD44Iw5qDiE9HIDXikjRkp5CJai96FQUIAaHVQs/hXXYMFbVwPY5++U0WOPlSdRMzxvo0+c+Mjs3XVXjItZ8OKzAtkdGo5eRVMbogXcz6OmpuM0Ts/u7WHD6Ux+Yp9uiIy/pFt/WOIMmIEw7jP8x941HWDbDsrOSZl78yONALbzqE/afXRns4WhmWt+5hLhmKzufU96/sCZbD1T0oNNWN7p6T25lrWpPuvRUBds5ZXbDTEvbgC5RRz9jZHybH0d6M4SvWiMme+Wp26ZAi/Q1fHE";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    static final int MOTOR_TICK_COUNTS = 145;


    public double shooterPos = .56;
    public boolean shooterReady = true;

    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    public RealRobot(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;


        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        ltrolley = hardwareMap.dcMotor.get("ltrolley");
        //rtrolley = hardwareMap.dcMotor.get("rtrolley");
        //carousel = hardwareMap.dcMotor.get("carousel");

        //lchain = hardwareMap.servo.get("lchain");
        //lclaw = hardwareMap.servo.get("lclaw");
        /*lclaw = hardwareMap.servo.get("lclaw");
        rclaw = hardwareMap.servo.get("rclaw");
        lchain = hardwareMap.servo.get("lchain");
        rchain = hardwareMap.servo.get("rchain");
        armLevel1 = armLevel0 - 100;
        armLevel2 = armLevel1 - 100;*/

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);//reverses the motors



        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, lf, lr, rf, rr);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, rf, rr, lr);



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //zeroPosition = slide.getCurrentPosition();
    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public VuforiaTrackables initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)


        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        return targetsSkyStone;
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }

    /**
     * @return true if the gyro is fully calibrated, false otherwise
     */
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _lf Left front motor
     * @param _lr Left rear motor
     * @param _rf Right front motor
     * @param _rr Right rear motor
     */

    public void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    public void driveInches(double speed, double inches){
        lf.setPower(speed);
        lr.setPower(speed);
        rf.setPower(speed);
        rr.setPower(speed);

        lf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        lr.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rr.setTargetPosition((int) (inches * COUNTS_PER_INCH));

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public String getVuforiaPosition(){

        targetVisible = false;
        //VuforiaTrackable trackable = allTrackables.get(0);
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                if (trackable.getName().equals("Stone Target")) {
                    telemetry.addLine("Stone Target is Visible");
                }

                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

            }
        }

        double xPosition = 0;
        double yPosition = 0;
        String positionSkystone = "";
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            xPosition = translation.get(0)/mmPerInch;
            yPosition = translation.get(1)/mmPerInch;
            if (yPosition < -7) {
                positionSkystone = "center";
            }else if (yPosition > -7){
                positionSkystone = "right";
            }


            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.addData("yPosition", yPosition);
        }
        else {
            positionSkystone = "left";
            telemetry.addData("Visible Target", "none");
        }
        telemetry.addData("Skystone Position", positionSkystone);
        telemetry.addData("yPosition: ", yPosition);
        telemetry.update();



        targetsSkyStone.deactivate();

        return positionSkystone;
    }


    public void stopRobot(){
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    public void drive(double speed) {
        stopRobot();
        setMotors(speed, -speed, speed, speed);

    }

    public void gyroDrive(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = speed * Math.sin(direction + Math.PI / 4.0);
        double rf = speed * Math.cos(direction + Math.PI / 4.0);
        double lr = speed * Math.cos(direction + Math.PI / 4.0);
        double rr = speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void speedGyroDrive(double x, double y, double robotSpeed) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, robotSpeed);

        double lf = speed * Math.sin(direction + Math.PI / 4.0);
        double rf = speed * Math.cos(direction + Math.PI / 4.0);
        double lr = speed * Math.cos(direction + Math.PI / 4.0);
        double rr = speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);

        telemetry.addData("RF & LR Speed", Math.cos(direction + Math.PI / 4.0));
        telemetry.update();
    }

    public void gyroDriveSlow(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = .5 * speed * Math.sin(direction + Math.PI / 4.0);
        double rf = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double lr = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double rr = .5 * speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void setUpGyroDrive(int distance){

        lf.setTargetPosition(distance + lf.getCurrentPosition());
        rf.setTargetPosition(distance + rf.getCurrentPosition());
        lr.setTargetPosition(distance + lr.getCurrentPosition());
        rr.setTargetPosition(distance + rr.getCurrentPosition());

    }

    /**
     *
     * @param power // power (Decimal) .0-1.0
     * @param distance // distance (in inches)
     * @param direction // direction (F = forward, B = back, L = Strafe left, R = Strafe Right)
     */

    public void encoderDrive(double power, double distance, char direction) {

        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // How many turns do I need the wheels to go [distance] inches?

        // The distance you drive with one turn of the wheel is the circumference of the wheel

        //Re-measure
        double circumference = ((direction == 'F' || direction == 'B') ? Math.PI*WHEEL_DIAMETER_INCHES : 11.4);
        //Gear ratio stuff :)
        double TICKS_PER_INCH = MOTOR_TICK_COUNTS/circumference;

        int eTarget = (int)(TICKS_PER_INCH*distance);



        ((DcMotorEx)lf).setTargetPositionTolerance(12);
        ((DcMotorEx)rf).setTargetPositionTolerance(12);
        ((DcMotorEx)lr).setTargetPositionTolerance(12);
        ((DcMotorEx)rr).setTargetPositionTolerance(12);

        if(direction == 'F')
        {
            lf.setTargetPosition(-eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(-eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'B')
        {
            lf.setTargetPosition(eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(-eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(-eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'L')
        {
            lf.setTargetPosition(eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'R')
        {
            lf.setTargetPosition(-eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(-eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(-eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(-eTarget + rr.getCurrentPosition());
        }

        //set the power desired for the motors
        lf.setPower(power*.7*(direction == 'R' || direction == 'F' ? 1.3 : 1));
        rf.setPower(power*.7*(direction == 'R' || direction == 'F' ? 1.3 : 1));
        lr.setPower(power*.7);
        rr.setPower(power*.7);

        // set the motors to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy())
        {
            loop();
            // make sure to not do anything while the motors are running
            telemetry.addData("Path", "Driving " + distance + " inches");
            /*telemetry.addData("Slide position:",slide.getCurrentPosition());
            telemetry.addData("Slide target:",slide.getTargetPosition());*/
            telemetry.addData("Current position", lf.getCurrentPosition());
            telemetry.addData("Target position", lf.getTargetPosition());
            telemetry.addData("Heading", getHeadingDegrees());



            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *
     * @param degrees //degrees (-360 to 360) you want to rotate (Positive is clockwise)
     */
    public void encoderRotate(int degrees, double power) {

        // RESETS ENCODERS

        // Circumference of the circle made by the robot (19-inch diameter * pi)
        double rotationLength = 34.5565;

        // Length the wheels would have to travel in order to rotate 1 degree in length (distance / 360)
        double degreeLength = rotationLength/360.0;

        double distance = Math.abs(degrees)*degreeLength;
        // The distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference = 3.14*WHEEL_DIAMETER_INCHES;

        double rotationsNeeded = distance/circumference;

        int eTarget = (int)(rotationsNeeded * MOTOR_TICK_COUNTS);

        ((DcMotorEx)lf).setTargetPositionTolerance(12);
        ((DcMotorEx)rf).setTargetPositionTolerance(12);
        ((DcMotorEx)lr).setTargetPositionTolerance(12);
        ((DcMotorEx)rr).setTargetPositionTolerance(12);
        //Set target position
        if(degrees > 0)
        {
            lf.setTargetPosition(eTarget    + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget*-1 + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget    + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget*-1 + rr.getCurrentPosition());
        }
        else if(degrees < 0)
        {
            lf.setTargetPosition(eTarget*-1 + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget    + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget*-1 + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget    + rr.getCurrentPosition());
        }

        //set the power desired for the motors
        lf.setPower(power*-.7);
        rf.setPower(power*-.7);
        lr.setPower(power*-.7);
        rr.setPower(power*-.7);

//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the motors to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy())
//        {
//            loop();
//            // make sure to not do anything while the motors are running
//            telemetry.addData("Path", "Driving " + distance + " inches");
//            telemetry.addData("Heading", getHeadingDegrees());
//            telemetry.update();
//        }



//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void mecanumEncoders()
    {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     @param degrees //degrees (-360 to 360) you want to rotate (relative to intial heading of robot)
     */
    public void rotate(int degrees, double power)
    {
        if(power >= 4) degrees-=(degrees>0 ? 2 : -2);

        if(getHeadingDegrees() > degrees)
        {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            do {
                loop();
                lf.setPower(-power);
                lr.setPower(power);
                rf.setPower(-power);
                rr.setPower(power);
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees-getHeadingDegrees()));
                telemetry.update();

            }while(getHeadingDegrees() - degrees > 2);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
        else if(getHeadingDegrees() < degrees)
        {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            do {
                loop();
                lf.setPower(power);
                lr.setPower(-power);
                rf.setPower(power);
                rr.setPower(-power);
                loop();
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees-getHeadingDegrees()));
                telemetry.update();

            }while(degrees - getHeadingDegrees() > 2);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
    }

    public void useEncoders()
    {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*public void outtake(int position)
    {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(.6);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(800);
    }*/
}

