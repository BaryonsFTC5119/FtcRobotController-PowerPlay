


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

@TeleOp(name = "Mecanum Testing", group = "Iterative OpMode")
public class MecanumTesting extends OpMode
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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.resetHeading();

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

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = controller.left_stick_y;
        double strafe = controller.left_stick_x;
        double twist  = controller.right_stick_x;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        robot.lf.setPower(speeds[0]);
        robot.rf.setPower(speeds[1]);
        robot.lr.setPower(speeds[2]);
        robot.rr.setPower(speeds[3]);

        if(controller.dpadUpOnce()){
            robot.lchain.setPosition(robot.lchain.getPosition()+0.05);
        }

        if(controller.dpadDownOnce()){
            robot.lchain.setPosition(robot.lchain.getPosition()-0.05);
        }

        if(controller.leftBumperOnce()){
            robot.lchain.setDirection(Servo.Direction.FORWARD);
        }

        if(controller.rightBumperOnce()){
            robot.lchain.setDirection(Servo.Direction.REVERSE);
        }

        /*if(controller.leftBumperOnce()) {
            robot.runUsingEncoders();
            robot.encoderDrive(.06,24.7,'B');
            robot.runWithoutEncoders();
        }
        if(controller.rightBumperOnce()) {
            robot.runUsingEncoders();
            robot.encoderDrive(.08,25,'F');
            robot.runWithoutEncoders();
        }*/




        //robot.setMotors(lf, lr, rf, rr);

        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());
        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);
        telemetry.addData("L Chain Position: ", robot.lchain.getPosition());
        telemetry.addData("Current servo direction: ", robot.lchain.getDirection());

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


