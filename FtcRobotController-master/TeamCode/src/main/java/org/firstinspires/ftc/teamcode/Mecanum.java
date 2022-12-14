/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

@TeleOp(name = "Mecanum")
public class Mecanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private Robot robot;
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
        robot = new Robot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);


        /* not connectedrobot.carriage.setPosition(.77);
        robot.carriage.setDirection(Servo.Direction.FORWARD);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    //reverse motors from what they are right now
    public void init_loop() {
        controller.update();
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        //telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     *
    @Override
    public void start() {
        runtime.reset();
        robot.resetHeading();
    }
    */

    public static double governor = 0.7;
    boolean heightCheck = false;
    boolean heightCheck2 = false;
    boolean dropped = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
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

        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", robot.getHeadingDegrees());

        final double x = Math.pow(controller.left_stick_x, 3.0);
        final double y = Math.pow(controller.left_stick_y*-1, 3.0);

        final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        final double direction = Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = (slowMode ? governor*.6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) + (slowMode ? .3 : 1)*rotation;
        double rf = (slowMode ? governor*.6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) - (slowMode ? .3 : 1)*rotation;
        double lr = (slowMode ? governor*.6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) + (slowMode ? .3 : 1)*rotation;
        double rr = (slowMode ? governor*.6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) - (slowMode ? .3 : 1)*rotation;

        // slowmode strafing is too slow, so this increases strafing speed while in slow mode
        if(slowMode && controller.left_stick_x != 0)
        {
                lf*=1.3;
                rf*=1.3;
                lr*=1.3;
                rr*=1.3;
        }

        if(controller.left_stick_x != 0)
        {
            lf*=1.2;
            rf*=1.2;
        }

        if(controller.right_stick_x != 0)
        {
            lf*=.7;
            rf*=.7;
            lr*=.7;
            rr*=.7;
        }

        robot.setMotors(-lf, lr, rf, -rr);

        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());

        if(controller2.right_trigger > .05) //changed to controller2
        {
            robot.intake.setPower(-.5);
        }
        else if(controller2.left_trigger > .05) //changed to controller2
        {
            robot.intake.setPower(.5);
        }
        else
        {
            robot.intake.setPower(0);
        }

        if(controller2.rightBumper())
        {
            robot.intake.setPower(-.6);
        }
        else if(controller2.leftBumper())
        {
            robot.intake.setPower(.6);
        }




        /*if(robot.lift.getCurrentPosition() > 400 && !heightCheck)
        {
            robot.carriage.setPosition(.65);
            heightCheck = true;
        }
        else if(robot.lift.getCurrentPosition() < 400 && heightCheck)
        {
            robot.carriage.setPosition(.77);
            heightCheck = false;
        }

        //moving slides up and down
        if(Math.abs(controller2.left_stick_y) > .01)
        {
            double slidePower = controller2.left_stick_y;

            if(controller2.left_stick_y < 0) slidePower = slidePower/2;
            else slidePower = slidePower/1.2;

            if(slidePower < 0 && robot.lift.getCurrentPosition() > 1800) slidePower = 0;

            if(slidePower > 0 && robot.lift.getCurrentPosition() < 0) slidePower = 0;
            robot.lift.setPower(-slidePower);
        }
        else
        {
            robot.lift.setPower(0);
        }
        */
        //dumping method
        /*if(controller2.AOnce())
        {
            if(robot.carriage.getPosition() > .73)
            {
                robot.carriage.setPosition(.25);
            }
            else if(robot.carriage.getPosition() < .73 && robot.carriage.getPosition() > .5)
            {
                robot.carriage.setPosition(.25);
            }
            else
                robot.carriage.setPosition(.77);

        }*/

        telemetry.addData("Lift position", robot.lift.getCurrentPosition());
        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);
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
