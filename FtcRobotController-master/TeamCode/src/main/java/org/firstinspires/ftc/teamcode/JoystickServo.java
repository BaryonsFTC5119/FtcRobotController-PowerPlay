package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.io.FileWriter;

@TeleOp(name = "JoystickServo", group = "Iterative OpMode")
public class JoystickServo extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lr = null; //used to be leftDrive
    private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean slowMode = false;
    private Servo lclaw;



    int Bcounter = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //robot = new RealRobot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lclaw = hardwareMap.servo.get("lclaw");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        lclaw.setPosition(0);

    }

    //public static double governor = 0.7;


    @Override
    public void loop() {
        controller.update();
        controller2.update();
        //robot.loop();


        if(controller.dpadUpOnce()){
            lclaw.setPosition(lclaw.getPosition()+0.1);
        }

        if(controller.dpadDownOnce()){
            lclaw.setPosition(lclaw.getPosition()-0.1);
        }

        telemetry.addData("Lclaw position:", lclaw.getPosition());
        telemetry.update();

    }

}