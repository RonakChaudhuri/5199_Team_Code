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

/**
 * ALL CONTROLS FOR GAMEPADS
 *
 * GAMEPAD ONE:
 *
 * LEFT TRIGGER: ----------         RIGHT TRIGGER: Turbo Speed
 * LEFT BUMPER: Platform Up(Manual) RIGHT BUMPER: Platform Down(Manual)
 * LEFT STICK: Move/Strafe          RIGHT STICK: Turn
 * X: -----------                   B: Platform Servo Data
 * Y: Platform Up                   A: Platform Down
 * D-PAD UP/DOWN: -------           D-PAD Right/Left: --------
 *
 *
 * GAMEPAD TWO:
 *
 * LEFT TRIGGER: Intake Open        RIGHT TRIGGER: Intake Close
 * LEFT BUMPER: --------            RIGHT BUMPER:  ---------
 * LEFT STICK: Lift Arm Up/Down     RIGHT STICK: -------
 * X: Grabber Forward               B: Grabber Back
 * Y: --------                      A: ---------
 * D-PAD UP/DOWN: ------            D-PAD Right/Left: --------
 *
 */

@TeleOp(name="TeleOp4", group="Linear Opmode")
//@Disabled
public class TeleOp4 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    //private DcMotor pivotMotor = null;
    //private DcMotor liftMotorRight = null;
    private DcMotor liftMotorLeft = null;
    private DcMotor rightIntakeMotor = null;
    private DcMotor leftIntakeMotor = null;
    private Servo platformServo = null;
    private Servo grabberServoRight = null;
    private Servo grabberServoLeft = null;
    //private Servo rightServo = null;
    //private CRServo actuatorServo = null;
    //private DcMotor actuatorMotor = null;
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
        //pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
        //actuatorServo = hardwareMap.get(CRServo.class, "actuator_servo");
        //actuatorMotor = hardwareMap.get(DcMotor.class, "actuator_motor");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "intake_right");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "intake_left");
        //liftMotorRight = hardwareMap.get(DcMotor.class, "lift_right");
        liftMotorLeft = hardwareMap.get(DcMotor.class, "lift_left");
        platformServo = hardwareMap.get(Servo.class, "platform_servo");
        //rightServo = hardwareMap.get(Servo.class, "servo_right");
        grabberServoLeft = hardwareMap.get(Servo.class, "grabber_servo_left");
        grabberServoRight = hardwareMap.get(Servo.class, "grabber_servo_right");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        //pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        //actuatorServo.setDirection(CRServo.Direction.REVERSE);
        //actuatorMotor.setDirection(DcMotor.Direction.REVERSE);
        //liftMotorRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        platformServo.setDirection(Servo.Direction.REVERSE);
        grabberServoRight.setDirection(Servo.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);


        platformServo.setPosition(0.3);
        grabberServoLeft.setPosition(0);
        grabberServoRight.setPosition(0);
       //rightServo.setPosition(0.3);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            double leftFrontPower;
            double leftRearPower;
            double rightFrontPower;
            double rightRearPower;
            double intakeMotorClosePower;
            double intakeMotorOpenPower;
            double liftMotorPower;
            double drive = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double turn  =  Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double strafe = gamepad1.right_stick_x;
            double motorOpen = gamepad2.left_trigger;
            double motorClose = gamepad2.right_trigger;
            double lift = gamepad2.left_stick_y;
            //pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            rightFrontPower = Range.clip(drive * Math.cos(turn) + strafe, -1.0, 1.0);
            rightRearPower = Range.clip(drive * Math.sin(turn) + strafe, -1.0, 1.0);
            leftFrontPower = Range.clip(drive * Math.sin(turn) - strafe, -1.0, 1.0);
            leftRearPower = Range.clip(drive * Math.cos(turn) - strafe, -1.0, 1.0);
            intakeMotorOpenPower = Range.clip(motorOpen, 0.0, 1.0);
            intakeMotorClosePower = Range.clip(motorClose, 0.0, 1.0);
            liftMotorPower = Range.clip(lift, -1.0, 1.0);


//            //Actuator Motor Controls
//            if(gamepad2.left_trigger > 0)
//            {
//                actuatorMotor.setPower(actuatorMotorOpenPower * 1.3);
//
//            }
//            actuatorMotor.setPower(0);
//            if(gamepad2.right_trigger > 0)
//            {
//                actuatorMotor.setPower(-actuatorMotorClosePower * 1.3);
//
//            }
//            actuatorMotor.setPower(0);


            //Intake Motor Controls
            if(gamepad2.left_trigger > 0)
            {
                leftIntakeMotor.setPower(intakeMotorOpenPower * 1.1);
                rightIntakeMotor.setPower(intakeMotorOpenPower * 1.1);
            }
            leftIntakeMotor.setPower(0);
            rightIntakeMotor.setPower(0);
            if(gamepad2.right_trigger > 0)
            {
                leftIntakeMotor.setPower(-intakeMotorClosePower * 1.1);
                rightIntakeMotor.setPower(-intakeMotorClosePower * 1.1);
            }
            leftIntakeMotor.setPower(0);
            rightIntakeMotor.setPower(0);



            //Drive Controls
            if (gamepad1.right_trigger > 0)
            {
                leftFront.setPower(leftFrontPower * 3);
                leftRear.setPower(leftRearPower * 3);
                rightFront.setPower(rightFrontPower * 3);
                rightRear.setPower(rightRearPower * 3);
            }
            if (gamepad1.right_bumper)
            {
                leftFront.setPower(leftFrontPower * .2);
                leftRear.setPower(leftRearPower * .2);
                rightFront.setPower(rightFrontPower * .2);
                rightRear.setPower(rightRearPower * .2);
            }

            leftFront.setPower(leftFrontPower * .8);
            leftRear.setPower(leftRearPower * .8);
            rightFront.setPower(rightFrontPower * .8);
            rightRear.setPower(rightRearPower * .8);






//            //Pivot Actuator Controls
//            if(gamepad2.b)
//            {
//                if(!turned)
//                {
//                    moveDistancePivot(.5, .0394 * 1.75);
//                    turned = true;
//                }
//            }
//            if(gamepad2.x)
//            {
//                if(turned)
//                {
//                    moveDistancePivot(.5, -.0394 * 1.75);
//                    turned = false;
//                }
//            }
//            if(gamepad2.left_bumper)
//            {
//                moveDistancePivot(.2, -.005);
//            }
//            if(gamepad2.right_bumper)
//            {
//                moveDistancePivot(.2, .005);
//            }


            //Grabber Servo Controls
            if(gamepad2.x)
            {
                grabberServoRight.setPosition(.1);
                grabberServoLeft.setPosition(.1);
            }
            if(gamepad2.b)
            {
                grabberServoRight.setPosition(0);
                grabberServoLeft.setPosition(0);
            }
            if(gamepad2.left_bumper)
            {
               grabberServoLeft.setPosition(grabberServoLeft.getPosition() + .01);
               grabberServoRight.setPosition(grabberServoRight.getPosition() + .01);
            }
            if(gamepad2.right_bumper)
            {
                grabberServoLeft.setPosition(grabberServoLeft.getPosition() - .01);
                grabberServoRight.setPosition(grabberServoRight.getPosition() - .01);
            }




            //Lift Motor controls
            if (gamepad2.left_stick_y > 0)
            {
                liftMotorLeft.setPower(liftMotorPower * 2);
                //liftMotorRight.setPower(liftMotorPower * 2);
            }
            liftMotorLeft.setPower(0);
            //liftMotorRight.setPower(0);

            if (gamepad2.left_stick_y < 0)
            {
                liftMotorLeft.setPower(liftMotorPower * 2);
                //liftMotorRight.setPower(liftMotorPower * 2);
            }
            liftMotorLeft.setPower(0);
            //liftMotorRight.setPower(0);




            //Platform Servo Controls
            if(gamepad1.a)
            {
                platformServo.setPosition(.69);
                //rightServo.setPosition(.69);
            }
            if(gamepad1.y)
            {
                platformServo.setPosition(0);
                //rightServo.setPosition(0);
            }
//            if(gamepad1.x)
//            {
//                //platformServo.setPosition(.46);
//                rightServo.setPosition(.46);
//            }
            if(gamepad1.b)
            {
                telemetry.addData("Servo Position Left", platformServo.getPosition());
                //telemetry.addData("Servo Position Right", rightServo.getPosition());
                telemetry.update();
            }
            if(gamepad1.right_bumper)
            {
                platformServo.setPosition(platformServo.getPosition() + .01);
                //rightServo.setPosition(rightServo.getPosition() + .01);
            }
            if(gamepad1.left_bumper)
            {
                platformServo.setPosition(platformServo.getPosition() - .01);
                //rightServo.setPosition(rightServo.getPosition() - .01);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), left(%.2f), right(%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
//    public void movePivot(double power)
//    {
//        pivotMotor.setPower(power);
//    }
//
//    public void stopRobot()
//    {
//        pivotMotor.setPower(0);
//    }
//    public void moveDistancePivot(double power, double distance)
//    {
//        pivotMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        int amountToMove = (int)(distance * COUNTS_PER_INCH );
//
//        pivotMotor.setTargetPosition(amountToMove);
//
//        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        movePivot(power);
//
//
//
//        while (pivotMotor.isBusy())
//        {
//            leftFront.setPower(0);
//            leftRear.setPower(0);
//            rightFront.setPower(0);
//            rightRear.setPower(0);
//
//        }
//
//        stopRobot();
//        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
}
