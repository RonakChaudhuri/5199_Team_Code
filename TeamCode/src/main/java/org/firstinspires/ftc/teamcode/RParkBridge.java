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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="RedParkBridgeSkystone", group="Linear Opmode")
//@Disabled
public class RParkBridge extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
   // private DcMotor actuatorMotor = null;
    //private DcMotor pivotMotor = null;
    //private Servo leftServo = null;
    //private Servo rightServo = null;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_MOTOR_REV_PIVOT    = 696.5; //235.2
    static final double     DRIVE_GEAR_REDUCTION_PIVOT    = 1.75;
    static final double     PIVOT_DIAMETER_INCHES   = .023622;
    static final double     COUNTS_PER_INCH_PIVOT      = (COUNTS_PER_MOTOR_REV_PIVOT * DRIVE_GEAR_REDUCTION_PIVOT) /
            (PIVOT_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MOTOR_REV_ACTUATOR = 145.6; //232.96    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ACTUATOR = 1;     // This is < 1.0 if geared UP
    static final double ACTUATOR_DIAMETER_INCHES = 1.953;     // For figuring circumference
    static final double COUNTS_PER_INCH_ACTUATOR = (COUNTS_PER_MOTOR_REV_ACTUATOR * DRIVE_GEAR_REDUCTION_ACTUATOR) /
            (ACTUATOR_DIAMETER_INCHES * 3.1415);

//5.2:1
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "left_rear");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear");
//        actuatorMotor = hardwareMap.get(DcMotor.class, "actuator_motor");
//        pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
//        leftServo = hardwareMap.get(Servo.class, "servo_left");
//        rightServo = hardwareMap.get(Servo.class, "servo_right");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
       // actuatorMotor.setDirection(DcMotor.Direction.REVERSE);
       // leftServo.setDirection(Servo.Direction.REVERSE);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftServo.setPosition(0);
//        rightServo.setPosition(0);
        waitForStart();
        runtime.reset();

        moveDistance(.6, 27);
        moveDistanceStrafe(.4, 10);




    }


    public void move(double power)
    {


        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


    }
    public void strafe(double power)
    {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        rightRearMotor.setPower(power);
    }

    public void stopRobot() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    public void turnLeft(double power) {

        leftFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);

    }

    public void turnRight(double power) {
        turnLeft(-power);
    }
//    public void movePivot(double power)
//    {
//        pivotMotor.setPower(power);
//    }

    public void moveDistance(double power, int distance) {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int) (distance * COUNTS_PER_INCH);


        leftFrontMotor.setTargetPosition(amountToMove);
        leftRearMotor.setTargetPosition(amountToMove);
        rightFrontMotor.setTargetPosition(amountToMove);
        rightRearMotor.setTargetPosition(amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        move(power);


        while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())
        {


        }

        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveDistanceStrafe(double power, double distance)
    {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int)(distance * COUNTS_PER_INCH);


        leftFrontMotor.setTargetPosition(amountToMove);
        leftRearMotor.setTargetPosition(-amountToMove);
        rightFrontMotor.setTargetPosition(-amountToMove);
        rightRearMotor.setTargetPosition(amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafe(power);


        while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())
        {


        }

        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnLeftDistance(double power, int distance) {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int) (distance * COUNTS_PER_INCH);

        leftFrontMotor.setTargetPosition(-amountToMove);
        leftRearMotor.setTargetPosition(-amountToMove);
        rightFrontMotor.setTargetPosition(amountToMove);
        rightRearMotor.setTargetPosition(amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnLeft(power);


        while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy()) {


        }

        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRightDistance(double power, int distance) {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int) (distance * COUNTS_PER_INCH);

        leftFrontMotor.setTargetPosition(amountToMove);
        leftRearMotor.setTargetPosition(amountToMove);
        rightFrontMotor.setTargetPosition(-amountToMove);
        rightRearMotor.setTargetPosition(-amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnRight(power);


        while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy()) {


        }

        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


//    public void moveActuator(double power)
//    {
//        actuatorMotor.setPower(power);
//
//    }
//    public void moveActuatorDistance(double power, double distance)
//    {
//        actuatorMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        int amountToMove = (int) (distance * COUNTS_PER_INCH_ACTUATOR);
//
//
//        actuatorMotor.setTargetPosition(amountToMove);
//
//        actuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        moveActuator(power);
//
//
//        while (actuatorMotor.isBusy())
//        {
//
//
//        }
//
//        stopRobot();
//        actuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//    public void moveDistancePivot(double power, double distance)
//    {
//        pivotMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        int amountToMove = (int)(distance * COUNTS_PER_INCH_PIVOT );
//
//        pivotMotor.setTargetPosition(amountToMove);
//
//        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        movePivot(power);
//
//
//        while (pivotMotor.isBusy())
//        {
//
//
//        }
//
//        stopRobot();
//        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

}
