
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.FileWriter;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MaybeMecanum", group = "Iterative OpMode")
public class MaybeMecanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lr = null; //used to be leftDrive
    private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean slowMode = false;
    Servo claw, lchain, openSesame;


    int Bcounter = 0;
    String state = "end";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RealRobot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lchain = hardwareMap.servo.get("lchain");
        claw = hardwareMap.servo.get("lclaw");
        openSesame = hardwareMap.servo.get("OS");
        lchain.scaleRange(0,0.6);
        claw.scaleRange(0.5, 1.0);
        claw.setPosition(0.5);
        robot.ltrolley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller.update();
        if (controller.AOnce()) {
            //arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.resetHeading();
        robot.ltrolley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ltrolley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.ltrolley.setTargetPosition(2000);

    }


    public static double governor = 0.7;
    boolean heightCheck = false;
    boolean heightCheck2 = false;
    boolean dropped = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //robot.ltrolley.setTargetPosition(2000);
        //robot.ltrolley.setPower(0.2);
        controller.update();
        controller2.update();
        robot.loop();

        if (controller.XOnce()) {
            robot.resetHeading();
        }
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        if (controller.BOnce()) {
            robot.encoderRotate(90, 0.4);
        }

        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", robot.getHeadingDegrees());

        final double x = Math.pow(controller.left_stick_x*1, 3.0);
        final double y = Math.pow(controller.left_stick_y*1, 3.0);

        final double rotation = -Math.pow(controller.right_stick_x*1, 3.0)/1.5;
        final double direction = -(Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0));
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = (slowMode ? governor*.6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) + (slowMode ? .3 : 1)*rotation;
        double rf = (slowMode ? governor*.6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) - (slowMode ? .3 : 1)*rotation;
        double lr = (slowMode ? governor*.6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) + (slowMode ? .3 : 1)*rotation;
        double rr = (slowMode ? governor*.6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) - (slowMode ? .3 : 1)*rotation;

        //top Y, left X, bottom A, right B
        if(controller2.X()){
            lchain.setPosition(0);
        }

        if(controller2.B()){
            lchain.setPosition(1);
        }

        if(controller2.AOnce()){
            lchain.setPosition(lchain.getPosition()-0.2);
        }

        if(controller2.YOnce()){
            lchain.setPosition(lchain.getPosition()+0.2);
        }

        if(controller2.left_trigger>0.2){
            claw.setPosition(claw.getPosition()+0.1);
            state="end";
        }

        if(controller2.right_trigger>0.2){
            claw.setPosition(claw.getPosition()-0.1);
            state="end";
        }

        if(controller2.left_stick_y<0){
            robot.ltrolley.setPower(0.8);
            state="end";
        }

        if(controller2.left_stick_y>0){
            //robot.ltrolley.setPower(-0.5);
            robot.ltrolley.setPower(-0.8*controller2.left_stick_y);
            state="end";
        }

        if(controller2.left_stick_y==0){
            robot.ltrolley.setPower(0);
        }

        if(controller2.leftBumper()){
            openSesame.setPosition(openSesame.getPosition()+0.1);
        }

        if(controller2.rightBumper()){
            openSesame.setPosition(openSesame.getPosition()-0.1);
        }
        if(controller2.dpadDownOnce()) robot.light.setPower(0.5);
        if(controller2.dpadRightOnce()) robot.light.setPower(1.0);

        if(controller2.dpadUpOnce()) {
            state = "lift"; //-461

        }
        if(state.equals("start")){
            if(robot.ltrolley.getCurrentPosition()>=-461){
                state="lift";

            }
        }
        if(state.equals("lift")){
            robot.ltrolley.setTargetPosition(2000);
            robot.ltrolley.setPower(1.0);
            lchain.setPosition(0.6);
            if(robot.ltrolley.getCurrentPosition()>=1900){
                //motor just stops

                //lclaw.setPosition(0.0);
                //lchain.setPosition(1.0);
                state="end";
                //robot.ltrolley.setPower(0);
            }
        }





        robot.setMotors(lf, lr, rf, rr);

        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());
        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);
        telemetry.addData("Lchain wants to go to: ", lchain.getPosition());
        telemetry.addData("claw wants to go to:  ", claw.getPosition());
        telemetry.addData("Open sesame wants to go to: ", openSesame.getPosition());
        telemetry.addData("LTrolley position: ", robot.ltrolley.getCurrentPosition());
        telemetry.addData("Light power", robot.light.getPower());
        telemetry.addData("Status ", state);

//        telemetry.addData("Lift target position", robot.lift.getTargetPosition());
//        telemetry.addData("Carriage Position", robot.carriage.getPosition());
//        telemetry.addData("Height check 1: ", (heightCheck ? "True" : "False"));
//        telemetry.addData("Height check 2: ", (heightCheck2 ? "True" : "False"));
//        telemetry.addData("dropped: ", (dropped ? "True" : "False"));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}