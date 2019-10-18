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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp2", group="Iterative Opmode")

public class TeleOp2 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor pivotMotor = null;
    private CRServo actuatorServo = null;
    private boolean isSlow;
    private boolean slowHeld;
    int xcount;
    int acount;
    private boolean turned = false;
    static final double     COUNTS_PER_MOTOR_REV    = 696.5; //235.2
    static final double     DRIVE_GEAR_REDUCTION    = 1.75;
    static final double     PIVOT_DIAMETER_INCHES   = .023622;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PIVOT_DIAMETER_INCHES * 3.1415);


    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
        actuatorServo = hardwareMap.get(CRServo.class, "actuator_servo");
        //lift_motor

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        actuatorServo.setDirection(CRServo.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {

        runtime.reset();

    }

    @Override
    public void loop()
    {
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;
        double actuatorServoClosePower;
        double actuatorServoOpenPower;
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double drive = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double turn  =  Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double strafe = gamepad1.right_stick_x;
        double servoClose = gamepad1.left_trigger;
        double servoOpen = gamepad1.right_trigger;


         rightFrontPower = Range.clip(drive * Math.cos(turn) + strafe, -1.0, 1.0);
         rightRearPower = Range.clip(drive * Math.sin(turn) + strafe, -1.0, 1.0);
         leftFrontPower = Range.clip(drive * Math.sin(turn) - strafe, -1.0, 1.0);
         leftRearPower = Range.clip(drive * Math.cos(turn) - strafe, -1.0, 1.0);
         actuatorServoClosePower = Range.clip(servoClose, 0.0, 1.0);
         actuatorServoOpenPower = Range.clip(servoOpen, 0.0, 1.0);


         //Actuator Servo Controls
          if(gamepad1.left_trigger > 0)
          {
             actuatorServo.setPower(actuatorServoClosePower);
          }
          if(gamepad1.right_trigger > 0)
          {
             actuatorServo.setPower(actuatorServoOpenPower);
          }


          //Drive Controls
          if (gamepad1.y)
           {
               leftFront.setPower(leftFrontPower * .2);
               leftRear.setPower(leftRearPower * .2);
               rightFront.setPower(rightFrontPower * .2);
               rightRear.setPower(rightRearPower * .2);
           }
          if (gamepad1.right_bumper)
          {
              leftFront.setPower(leftFrontPower);
              leftRear.setPower(leftRearPower);
              rightFront.setPower(rightFrontPower);
              rightRear.setPower(rightRearPower);
          }

        leftFront.setPower(leftFrontPower * .8);
        leftRear.setPower(leftRearPower * .8);
        rightFront.setPower(rightFrontPower * .8);
        rightRear.setPower(rightRearPower * .8);



        //Pivot Actuator Controls
        if(gamepad1.b)
        {
            if(!turned)
            {

                    moveDistance(.5, .0394 * 1.75);

                turned = true;
            }
        }
        if(gamepad1.x)
        {
            if(turned)
           {

                    moveDistance(.5, -.0394 * 1.75);


                turned = false;
            }
        }
        if(gamepad1.left_bumper)
        {
            moveDistance(.2, -.005);


        }
        if(gamepad1.right_bumper)
        {
            moveDistance(.2, .005);
        }





        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), left(%.2f), right(%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    public void move(double power)
    {
        pivotMotor.setPower(power);
    }

    public void stopRobot()
    {
        pivotMotor.setPower(0);
    }
    public void moveDistance(double power, double distance)
    {
        pivotMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int)(distance * COUNTS_PER_INCH );

        pivotMotor.setTargetPosition(amountToMove);

        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        move(power);


        while (pivotMotor.isBusy())
        {


        }

        stopRobot();
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop()
    {

    }

}
