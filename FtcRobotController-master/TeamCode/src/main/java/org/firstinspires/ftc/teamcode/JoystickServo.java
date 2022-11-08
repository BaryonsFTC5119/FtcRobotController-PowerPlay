package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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


    int Bcounter = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RealRobot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        controller.update();
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();
    }

     @Override
    public void start() {
        runtime.reset();
        robot.resetHeading();
        lchain.setPosition(0.0);

    }

    public static double governor = 0.7;
    boolean heightCheck = false;
    boolean heightCheck2 = false;
    boolean dropped = false;

    @Override
    public void loop() {
        controller.update();
        controller2.update();
        robot.loop();


        if(controller.dpadupOnce()&&lchain.getPosition()<1){
            lchain.setPosition(lchain.getPosition()+0.05);
        }

        if(controller.dpaddownOnce()&&lchain.getPosition()!=0){
            lchain.setPosition(lchain.getPosition()-0.05)
        }

        telemetry.addData("L chain position: " + lchain.getPosition());
    }

}