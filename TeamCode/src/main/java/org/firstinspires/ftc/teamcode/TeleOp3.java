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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp3", group="Linear Opmode")
//@Disabled
public class TeleOp3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor pivotMotor = null;
    private DcMotor liftMotor = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    //private CRServo actuatorServo = null;
    private DcMotor actuatorMotor = null;
    private boolean turned = false;
    static final double     COUNTS_PER_MOTOR_REV_PIVOT    = 696.5; //235.2
    static final double     DRIVE_GEAR_REDUCTION_PIVOT    = 1.75;
    static final double     PIVOT_DIAMETER_INCHES   = .023622;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV_PIVOT * DRIVE_GEAR_REDUCTION_PIVOT) /
            (PIVOT_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
        //actuatorServo = hardwareMap.get(CRServo.class, "actuator_servo");
        actuatorMotor = hardwareMap.get(DcMotor.class, "actuator_motor");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        leftServo = hardwareMap.get(Servo.class, "servo_left");
        rightServo = hardwareMap.get(Servo.class, "servo_right");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        //actuatorServo.setDirection(CRServo.Direction.REVERSE);
        actuatorMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);


        leftServo.setPosition(0);
        rightServo.setPosition(0);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            double leftFrontPower;
            double leftRearPower;
            double rightFrontPower;
            double rightRearPower;
            double actuatorMotorClosePower;
            double actuatorMotorOpenPower;
            double liftMotorPower;
            double drive = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double turn  =  Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double strafe = gamepad1.right_stick_x;
            double motorOpen = gamepad2.left_trigger;
            double motorClose = gamepad2.right_trigger;
            double lift = gamepad2.left_stick_y;
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            rightFrontPower = Range.clip(drive * Math.cos(turn) + strafe, -1.0, 1.0);
            rightRearPower = Range.clip(drive * Math.sin(turn) + strafe, -1.0, 1.0);
            leftFrontPower = Range.clip(drive * Math.sin(turn) - strafe, -1.0, 1.0);
            leftRearPower = Range.clip(drive * Math.cos(turn) - strafe, -1.0, 1.0);
            actuatorMotorOpenPower = Range.clip(motorOpen, 0.0, 1.0);
            actuatorMotorClosePower = Range.clip(motorClose, 0.0, 1.0);
            liftMotorPower = Range.clip(lift, -1.0, 1.0);


            //Actuator Motor Controls
            if(gamepad2.left_trigger > 0)
            {
                actuatorMotor.setPower(actuatorMotorOpenPower * .9);

            }
            actuatorMotor.setPower(0);
            if(gamepad2.right_trigger > 0)
            {
                actuatorMotor.setPower(-actuatorMotorClosePower * .9);

            }
            actuatorMotor.setPower(0);




            //Drive Controls
            if (gamepad1.right_trigger > 0)
            {
                leftFront.setPower(leftFrontPower * 1.5);
                leftRear.setPower(leftRearPower * 1.5);
                rightFront.setPower(rightFrontPower * 1.5);
                rightRear.setPower(rightRearPower * 1.5);
            }
            if (gamepad1.x)
            {
                leftFront.setPower(leftFrontPower * .05);
                leftRear.setPower(leftRearPower * .05);
                rightFront.setPower(rightFrontPower * .05);
                rightRear.setPower(rightRearPower * .05);
            }

            leftFront.setPower(leftFrontPower * .9);
            leftRear.setPower(leftRearPower * .9);
            rightFront.setPower(rightFrontPower * .9);
            rightRear.setPower(rightRearPower * .9);






            //Pivot Actuator Controls
            if(gamepad2.b)
            {
                if(!turned)
                {
                    moveDistancePivot(.5, .0394 * 1.75);
                    turned = true;
                }
            }
            if(gamepad2.x)
            {
                if(turned)
                {
                    moveDistancePivot(.5, -.0394 * 1.75);
                    turned = false;
                }
            }
            if(gamepad2.left_bumper)
            {
                moveDistancePivot(.2, -.005);
            }
            if(gamepad2.right_bumper)
            {
                moveDistancePivot(.2, .005);
            }





            //Lift Motor controls
            if (gamepad2.left_stick_y > 0)
            {
                liftMotor.setPower(liftMotorPower * 1.2);
            }
            liftMotor.setPower(0);

            if (gamepad2.left_stick_y < 0)
            {
                liftMotor.setPower(liftMotorPower * 1.2);
            }
            liftMotor.setPower(0);


            if(gamepad1.b)
            {
                telemetry.addData("Servo Position Left", leftServo.getPosition());
                telemetry.addData("Servo Position Right", rightServo.getPosition());
                telemetry.update();
            }
            //Platform Servo Controls

            if(gamepad1.a)
            {
                leftServo.setPosition(.78);
                rightServo.setPosition(.5);
            }
            if(gamepad1.y)
            {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), left(%.2f), right(%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
    public void move(double power)
    {
        pivotMotor.setPower(power);
    }

    public void stopRobot()
    {
        pivotMotor.setPower(0);
    }
    public void moveDistancePivot(double power, double distance)
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
}
