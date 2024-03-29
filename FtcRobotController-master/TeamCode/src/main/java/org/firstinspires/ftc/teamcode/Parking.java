package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.*;

@Autonomous(name = "Cone parking")
public class Parking extends LinearOpMode {
    RealRobot robot;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/PowerPlayV2.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    Servo lclaw, lchain, openSesame;

    private static final String[] LABELS = {
            "1", // banana
            "2", // whale
            "3" // pencil
    };

    int coneNum;
    boolean detected = false;

    private static final String VUFORIA_KEY =
            "AQrfl2X/////AAABmXgYPrwW30wdoBAHztdXSDgfzAtx/3aneWzwyCjSRj16HSy" +
                    "36qQ27HjnVGpjTF+XeOKXM5S3kX+HHUnC8HDJBAD44Iw5qDiE9HIDXikjR" +
                    "kp5CJai96FQUIAaHVQs/hXXYMFbVwPY5++U0WOPlSdRMzxvo0+c+Mjs3XVXj" +
                    "ItZ8OKzAtkdGo5eRVMbogXcz6OmpuM0Ts/u7WHD6Ux+Yp9uiIy/pFt/WOIMmIE" +
                    "w7jP8x941HWDbDsrOSZl78yONALbzqE/afXRns4WhmWt+5hLhmKzufU96/sCZbD1T0o" +
                    "NNWN7p6T25lrWpPuvRUBds5ZXbDTEvbgC5RRz9jZHybH0d6M4SvWiMme+Wp26ZAi/Q1fHE";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        initialize();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()&&!detected) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                            if(recognition.getLabel().equals("1")){
                                detected = true;
                                coneNum = 1;
                            }

                            if(recognition.getLabel().equals("2")) {
                                detected = true;
                                coneNum = 2;
                            }

                            if(recognition.getLabel().equals("3")){
                                detected = true;
                                coneNum = 3;
                            }
                        }



                        telemetry.update();
                    }
                }
            }
        }
        lclaw.setPosition(0);
        robot.encoderDrive(0.7, 44.5, 'B');
        robot.rotate(241, 0.6);//122, 134.9
        openSesame.setPosition(1.0);
        sleep(2000);
        lclaw.setPosition(0);
        lchain.setPosition(0.2);
        robot.ltrolley.setTargetPosition(robot.ltrolley.getCurrentPosition()+3165);
        sleep(500);
        //lchain.setPosition(0.1);
        sleep(500);
        robot.ltrolley.setPower(-0.7);
        sleep(1000);
        robot.ltrolley.setPower(0.0);
        robot.ltrolley.setPower(0.7);
        sleep(2450);
        robot.encoderDrive(0.7, 1, 'B');
        robot.ltrolley.setPower(0.0);
        openSesame.setPosition(0.0);
        lclaw.setPosition(0);
        lchain.setPosition(0.558); // 0.6
        sleep(2900);
        lclaw.setPosition(1);
        sleep(500);
        lchain.setPosition(0.2);
        sleep(100);
        robot.rotate(70, 0.7);
        sleep(2000);
        robot.encoderDrive(0.5, 9, 'B');
        if(coneNum==1) {
            robot.encoderDrive(0.5, 7, 'B');
            robot.encoderDrive(0.5, 26, 'L');
            robot.encoderDrive(0.5, 3, 'F');
        }
        if(coneNum==3){
            robot.encoderDrive(0.5, 7, 'B');
            robot.encoderDrive(0.5, 28, 'R');
            robot.encoderDrive(0.5, 3, 'F');
        }

/*
        //openSesame.setPosition(0.0);
        //drive forward, rotate, state machine
        lclaw.setPosition(0.0);
        robot.encoderDrive(0.7, 50.0, 'B'); //52.0
        robot.rotate(90, 1.0);
        robot.encoderDrive(0.3, 12.0, 'B');
        //robot.rotate(-90,0.3);
        //robot.rotate(90,0.3);
      sleep(1000);
        openSesame.setPosition(1.0);
        robot.encoderDrive(0.3, 12.0, 'B');
        robot.ltrolley.setTargetPosition(650);
        lclaw.setPosition(0.0);
        lchain.setPosition(0.45);
//        robot.rotateToHeading(-100, 1.0);
        robot.rotate(-100,0.3);
        sleep(1000);
          robot.encoderDrive(0.8,3.0,'R');
        robot.ltrolley.setPower(-0.7);
        sleep(1000);
        robot.ltrolley.setPower(0.0);
        robot.ltrolley.setPower(0.7);
        sleep(2450);
        robot.ltrolley.setPower(0.0);
        lchain.setPosition(0.55);
        lclaw.setPosition(1.0);
        sleep(2000);
        lchain.setPosition(0.45);
        sleep(1000);
        robot.ltrolley.setTargetPosition(50);
        robot.ltrolley.setPower(-0.7);
        sleep(2500);
        robot.ltrolley.setPower(0);
        lchain.setPosition(0.45);
        lclaw.setPosition(0.0);
        sleep(1000);
        lchain.setPosition(0.55);
        robot.ltrolley.setTargetPosition(650);
        robot.ltrolley.setPower(0.7);
        sleep(2450);
        robot.ltrolley.setPower(0.0);
        lclaw.setPosition(1.0);

*/
        //robot.encoderDrive(0.3, 6, 'R');
        //robot.encoderDrive(0.3, 6.0, 'B');
        //robot.rotateToHeading(-148, 0.8);
        //drop cone
        //robot.ltrolley.setTargetPosition(2000);
        //robot.ltrolley.setPower(0.8);
        //lchain.setPosition(0.8);
        //robot.encoderDrive(0.3, 5, 'B');
        //8lchain.setPosition(0.6);
//        lclaw.setPosition(1.0);
//        sleep(500);
//        lchain.setPosition(0.0);
//        robot.rotateToHeading(180, 1.0);
//        robot.encoderDrive(0.3, 40.0, 'B');
//        /*robot.encoderDrive(0.3, 80, 'F'); //LRFB RLBF
//        robot.encoderDrive(0.3, 80, 'B');
//        robot.encoderDrive(0.3, 50, 'L');
//        robot.encoderDrive(0.3, 50, 'R');
//        //robot.encoderRotate(360, 0.5);
//        robot.rotateToHeading(60, 0.3);*/
//        if(coneNum==1){
//            robot.encoderDrive(0.3, 20.0, 'L');
//        }
//
//        if(coneNum==3){
//            robot.encoderDrive(0.3, 20.0, 'R');
//            //robot.rotateToHeading(0, 1.0);
//        }
//
//
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public void initialize() {
        ArrayList<DcMotor> allMotors = new ArrayList<>();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new RealRobot(hardwareMap, telemetry);
        allMotors.add(robot.lf);
        allMotors.add(robot.rf);
        allMotors.add(robot.lr);
        allMotors.add(robot.rr);

        //robot.carriage.setPosition(.77);
        //robot.carriage.setDirection(Servo.Direction.FORWARD);

        for (DcMotor dcMotor : allMotors) {
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot.loop();
        lchain = hardwareMap.servo.get("lchain");
        lclaw = hardwareMap.servo.get("lclaw");
        openSesame = hardwareMap.servo.get("OS");
        //lchain.scaleRange(0,0.8);
        /*robot.ltrolley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);*/
        robot.resetHeading();
        lclaw.setPosition(0);


    }
}