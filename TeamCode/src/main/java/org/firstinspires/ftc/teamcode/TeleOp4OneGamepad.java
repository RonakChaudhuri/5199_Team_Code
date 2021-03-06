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
 * LEFT TRIGGER: Intake Open/Lift Down      RIGHT TRIGGER: Intake Close/Lift Up
 * LEFT BUMPER: V4B back                    RIGHT BUMPER: V4B Forward
 * LEFT STICK: Move/Strafe                  RIGHT STICK: Turn
 * X: Switches To Lift Mode                 B: Open Claw
 * Y: Switches To Intake Mode               A: Close Claw
 * D-PAD UP/DOWN: Platform Up/Down          D-PAD Right/Left: --------
 *
 *
 *
 *
 *
 */

/*
SERVO PORTS ARE FLIPPED:
Left Servo in port 0
Right Servo in port 1
 */

@TeleOp(name="TeleOpOneGamepad", group="Linear Opmode")
//@Disabled
public class TeleOp4OneGamepad extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor liftMotorRight = null;
    private DcMotor liftMotorLeft = null;
    private DcMotor rightIntakeMotor = null;
    private DcMotor leftIntakeMotor = null;
    private Servo platformServo = null;
    private Servo grabberServoRight = null;
    private Servo grabberServoLeft = null;
    private Servo clawServo = null;
    private boolean IntakeMode = true;
    private boolean LiftMode = false;
//    private boolean open = true;
//    private boolean pressed = false;
    //private DigitalChannel limitSwitch = null;
//    private boolean turned = false;
    static final double     COUNTS_PER_MOTOR_REV_PIVOT    = 696.5; //235.2
    static final double     DRIVE_GEAR_REDUCTION_PIVOT    = 1.75;
    static final double     PIVOT_DIAMETER_INCHES   = .023622;
    static final double     COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV_PIVOT * DRIVE_GEAR_REDUCTION_PIVOT) /
                                                    (PIVOT_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Initialized");


        /**Hardware Map*/
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "intake_right");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "intake_left");
        liftMotorRight = hardwareMap.get(DcMotor.class, "lift_right");
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft = hardwareMap.get(DcMotor.class, "lift_left");
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        platformServo = hardwareMap.get(Servo.class, "platform_servo");
        grabberServoRight = hardwareMap.get(Servo.class, "grabber_servo_right");
        grabberServoLeft = hardwareMap.get(Servo.class, "grabber_servo_left");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_switch");


        /**Directions and Positions*/
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        grabberServoRight.setDirection(Servo.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        platformServo.setPosition(0);
        grabberServoLeft.setPosition(0.79);
        grabberServoRight.setPosition(0.79);
        clawServo.setPosition(0.69);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            double leftFrontPower;
            double leftRearPower;
            double rightFrontPower;
            double rightRearPower;
            double intakeMotorClosePowerG1;
            double intakeMotorOpenPowerG1;
            double liftMotorPowerDown;
            double liftMotorPowerUp;
            double drive = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double turn  =  Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double strafe = gamepad1.right_stick_x;
            double motorOpenG1 = gamepad1.left_trigger;
            double motorCloseG1 = gamepad1.right_trigger;
            double liftDown = Math.pow(gamepad1.left_trigger, 3);
            double liftUp =   Math.pow(gamepad1.right_trigger, 3);
            double grabberServoLeftPos = grabberServoLeft.getPosition();
            double grabberServoRightPos = grabberServoRight.getPosition();

            rightFrontPower = Range.clip(drive * Math.cos(turn) + strafe, -1.0, 1.0);
            rightRearPower = Range.clip(drive * Math.sin(turn) + strafe, -1.0, 1.0);
            leftFrontPower = Range.clip(drive * Math.sin(turn) - strafe, -1.0, 1.0);
            leftRearPower = Range.clip(drive * Math.cos(turn) - strafe, -1.0, 1.0);
            intakeMotorOpenPowerG1 = Range.clip(motorOpenG1, 0.0, 1.0);
            intakeMotorClosePowerG1 = Range.clip(motorCloseG1, 0.0, 1.0);
            liftMotorPowerDown = Range.clip(liftDown, -1.0, 1.0);
            liftMotorPowerUp = Range.clip(liftUp, -1.0, 1.0);


            /**GAMEPAD1 INTAKE*/
            if(IntakeMode)
            {
                if (gamepad1.left_trigger > 0)
                {
                    leftIntakeMotor.setPower(-intakeMotorOpenPowerG1 * 0.75);
                    rightIntakeMotor.setPower(-intakeMotorOpenPowerG1 * 0.75);
                }
                else if (gamepad1.right_trigger > 0)
                {
                    leftIntakeMotor.setPower(intakeMotorClosePowerG1);
                    rightIntakeMotor.setPower(intakeMotorClosePowerG1);
                } else
                    {
                    leftIntakeMotor.setPower(0);
                    rightIntakeMotor.setPower(0);
                }
            }
            if(gamepad1.y)
            {
                IntakeMode = true;
                LiftMode = false;
                telemetry.addLine("Intake Mode");
                telemetry.update();
            }
            if(gamepad1.x)
            {
                LiftMode = true;
                IntakeMode = false;
                telemetry.addLine("Lift Mode");
                telemetry.update();
            }

            /**DRIVE CONTROLS*/
            if (gamepad1.right_bumper)
            {
                leftFront.setPower(leftFrontPower * .01);
                leftRear.setPower(leftRearPower * .01);
                rightFront.setPower(rightFrontPower * .01);
                rightRear.setPower(rightRearPower * .01);
            }
            else
            {
                leftFront.setPower(leftFrontPower * 0.95);
                leftRear.setPower(leftRearPower * 0.95);
                rightFront.setPower(rightFrontPower * 0.95);
                rightRear.setPower(rightRearPower * 0.95);
            }

            if(gamepad1.right_stick_x != 0)
            {
                leftFront.setPower(leftFrontPower * .7);
                leftRear.setPower(leftRearPower * .7);
                rightFront.setPower(rightFrontPower * .7);
                rightRear.setPower(rightRearPower * .7);
            }



            /**V4B CONTROLS*/
            if(gamepad1.right_bumper)
            {
                telemetry.addData("Right Grabber Servo Position", grabberServoRight.getPosition());
                telemetry.addData("Left Grabber Servo Position", grabberServoLeft.getPosition());
                telemetry.update();
                grabberServoRight.setPosition(0.03);
                grabberServoLeft.setPosition(0.03);
            }
            if(gamepad1.left_bumper)
            {
                grabberServoRight.setPosition(0.79);
                grabberServoLeft.setPosition(0.79);
            }

//            if(gamepad2.left_trigger > 0 && grabberServoRightPos <= 0.77 && grabberServoLeftPos <= 0.77)
//            {
//                grabberServoRight.setPosition(grabberServoRightPos + 0.01);
//                grabberServoLeft.setPosition(grabberServoRightPos + 0.01);
//            }

//            if(gamepad2.right_trigger > 0)
//            {
//                grabberServoRight.setPosition(grabberServoRightPos - 0.01);
//                grabberServoLeft.setPosition(grabberServoRightPos - 0.01);
//            }


//            if (gamepad2.a && !open && !pressed)
//            {
//                clawServo.setPosition (.55);
//                open = true;
//                pressed = true;
//            }
//            pressed = false;
//            if (gamepad2.a && open && !pressed)
//            {
//                clawServo.setPosition (0.4);
//                open = false;
//                pressed = true;
//            }
//            pressed = false;

            /**CLAW*/
            if (gamepad1.a)
            {
                clawServo.setPosition (0.8);
            }
            if (gamepad1.b)
            {
                clawServo.setPosition (0.69);
            }
//            if(gamepad1.b)
//            {
//                clawServo.setPosition(.92);
//            }

            /**LIFT*/
            if(LiftMode)
            {
                if (gamepad1.left_trigger > 0)
                {
                    liftMotorLeft.setPower(-liftMotorPowerDown * 0.7);
                    liftMotorRight.setPower(-liftMotorPowerDown * 0.7);
                }
                else if (gamepad1.right_trigger > 0)
                {
                    liftMotorLeft.setPower(liftMotorPowerUp * 0.95 + 0.40);
                    liftMotorRight.setPower(liftMotorPowerUp * 0.95 + 0.40);
                }
                else
                {
                    liftMotorLeft.setPower(0); // -0.2 , remove braking, add encoder bounds
                    liftMotorRight.setPower(0);
                }
            }


            /**PLATFORM*/
            if(gamepad1.dpad_down)
            {
                platformServo.setPosition(.365);

            }
            if(gamepad1.dpad_up)
            {
                platformServo.setPosition(0);
            }
//            if(gamepad1.right_bumper)
//            {
//                platformServo.setPosition(platformServo.getPosition() + .001);
//            }
//            if(gamepad1.left_bumper)
//            {
//                platformServo.setPosition(platformServo.getPosition() - .001);
//            }



            /**TELEMETRY*/
            // Show the elapsed game time and wheel power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), right front(%.2f), left rear(%.2f), right rear(%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);

            if(platformServo.getPosition() == .365)
            {
                telemetry.addLine("Platform Down");
            }
            if(platformServo.getPosition() == 0)
            {
                telemetry.addLine("Platform Up");
            }
            telemetry.addData("Platform Servo Position: ", platformServo.getPosition());

            if(clawServo.getPosition() == .69)
            {
                telemetry.addLine("Claw Open");
            }
            if(clawServo.getPosition() == .8)
            {
                telemetry.addLine("Claw Closed");
            }
            if(clawServo.getPosition() == .92)
            {
                telemetry.addLine("Feeder Bot Activated");
            }
            telemetry.addData("Claw Servo Position: ", clawServo.getPosition());

//            if(limitSwitch.getState())
//            {
//                telemetry.addLine("Limit Switch Pressed");
//            }
            telemetry.update();
        }
    }

}
