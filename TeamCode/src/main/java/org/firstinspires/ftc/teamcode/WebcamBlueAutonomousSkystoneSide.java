/* Copyright (c) 2019 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Blue Skystone", group = "Concept")
//@Disabled
public class WebcamBlueAutonomousSkystoneSide extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor rightIntakeMotor = null;
    private DcMotor leftIntakeMotor = null;
    boolean detected = false;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_MOTOR_REV_PIVOT   = 696.5; //235.2
    static final double     DRIVE_GEAR_REDUCTION_PIVOT    = 1.75;
    static final double     PIVOT_DIAMETER_INCHES_PIVOT  = .023622;
    static final double     COUNTS_PER_INCH_PIVOT         = (COUNTS_PER_MOTOR_REV_PIVOT * DRIVE_GEAR_REDUCTION_PIVOT) /
            (PIVOT_DIAMETER_INCHES_PIVOT * 3.1415);
    private static final String VUFORIA_KEY =
            "AVlh/fr/////AAABmbDOVEeXhEIvtSZdmDAQFwpoeLbt2JNrdnl5vpfaSvtzRn2Hzjlh9tTlGfT35TawMAY9hmptf7PzZU4j99x0PX1xFsgc1xIbWGkAzFO6R5Zt42M/povDKHMbbUlgVarwjfyTZr3lcN+m3cU29zTj6zkie5n1q+GhG56whrVsTaWzt7oaZIr+0KIjFzHDfCOWQr9NB1C/jrKkrQT0hR48pbvpZO7t4t/fuCmB0Xp9Bji6T3HG2COQRYV8wThl3HjXJLadeU/Bh6jnOsPgH60FnOiCCnhGzdlhk3ccserQH7UPNnLJS1EaWaFG8n3wH09iLUiF3H56XFO/BbG1sD8RkIFfOT1NThzgOb2HaQjOndHw";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    boolean skystone = false;
    boolean stone = false;
    boolean webcam = false;
    boolean firstIteration = true;
    int count = 0;
    int index = 0;



    @Override
    public void runOpMode()
    {
        initVuforia();
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front");
        leftRearMotor  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightRearMotor  = hardwareMap.get(DcMotor.class, "right_rear");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "intake_right");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "intake_left");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null)
        {
            tfod.activate();
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();


        sleep(1000);

        if (opModeIsActive())
        {
            tfod.shutdown();
            moveDistance(.8, -13);

            initTfod();
            tfod.activate();
            while (opModeIsActive() && count < 1)
            {
                if (tfod != null)
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null)
                    {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions)
                        {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();
                            if(updatedRecognitions.size() > 0)
                            {
                                detected = true;
//                                if(firstIteration)
//                                {
//                                    moveDistance(.5, -15);
//                                    //moveDistanceStrafe(0.5, -4, 700);
//                                    firstIteration = false;
//                                    sleep(1000);
//                                    webcam = true;
//                                }
                                if(recognition.getLabel().equals("Stone") && count ==0 || ((recognition.getLabel().equals("Skystone")  && recognition.getLabel().equals("Stone"))))
                                {
                                    telemetry.addLine("Stone");
                                    telemetry.update();
                                    moveDistanceStrafe(1.3, -11.6, 800);
                                    turnLeftDistance(.8, 1.5, 400);

                                    //turnRightDistance(.5, 1);
                                    index++;
                                    if(index == 2)
                                    {
                                        moveDistance(.5, -3);
                                        moveDistanceStrafe(.5, -.8, 300);
                                    }
                                    if(index >4)
                                    {
                                        sleep(30000);
                                    }


                                }
//
                                else if(recognition.getLabel().equals("Skystone") && count < 1 && !recognition.getLabel().equals("Stone"))
                                {
                                    skystone =  true;
                                    telemetry.addLine("Skystone     ");
                                    telemetry.update();
                                    moveDistance(.5, -18); //forward
                                    leftIntakeMotor.setPower(-2);
                                    rightIntakeMotor.setPower(-2);
                                    moveDistance(.5, -4); //forward
                                    sleep(1000);//pickup
                                    leftIntakeMotor.setPower(0);
                                    rightIntakeMotor.setPower(0);
                                    //moveDistance(.3, 12);  //backward
                                    moveDistance(.5, 10);
                                    turnLeftDistance(.8, 23, 800);
                                    moveDistance(.8, -28 - (11.7*index)); //-35
                                    //input placing code
                                    sleep(1000);                  //drop off
                                    if(index == 0)
                                    {

                                        moveDistance(.8, 28); //35 move to the backmost stone
                                        turnRightDistance(.5, 23, 1500);
                                        moveDistanceStrafe(1.3, -34.8, 1500);
                                        //turnLeftDistance(.8, 1, 200);
                                        //moveDistanceStrafe(1.3, -10.0, 500);
                                        //turnLeftDistance(.8, 1, 200);
                                        //moveDistanceStrafe(1.3, -10.0, 500);
                                        turnLeftDistance(.8, 2, 500);
                                        moveDistance(.8, -14); //forward
                                        leftIntakeMotor.setPower(-2);
                                        rightIntakeMotor.setPower(-2);
                                        moveDistance(.5, -4); //forward
                                        sleep(1000);
                                        moveDistance(.5, 10);
                                        turnLeftDistance(.8, 23,800);
                                        index = 3;
                                        moveDistance(.5, -30 - (11.6*index));//-35
                                        //input placement code
                                        sleep(1000);                  //drop off
                                        moveDistance(.8, 8);

                                    }
                                    else if(index == 1)
                                    {

                                        moveDistance(.8, 39.77); //46.77 move to the backmost stone
                                        turnRightDistance(.5, 23, 1500);
                                        moveDistanceStrafe(1.3, -34.8, 1500);
                                        //turnLeftDistance(.5, 1, 500);
                                        //moveDistanceStrafe(1.3, -10.1, 1200);
                                        //turnLeftDistance(.5, 1, 500);
                                       // moveDistanceStrafe(.5, -10.1, 1200);
                                        turnLeftDistance(.8, 2, 500);
                                        moveDistance(.5, -14); //forward
                                        leftIntakeMotor.setPower(-2);
                                        rightIntakeMotor.setPower(-2);
                                        moveDistance(.5, -4); //forward
                                        sleep(1000);
                                        moveDistance(.5, 10);
                                        turnLeftDistance(.5, 23,1500);
                                        moveDistanceStrafe(.8, 14, 800);
                                        index = 4;
                                        moveDistance(.5, -30 - (11.6*index));//-35
                                        //input placement code
                                        sleep(1000);                  //drop off
                                        moveDistance(.8, 8);

                                    }
                                    else
                                    {
                                        moveDistance(.5, 8);
                                    }



                                    //moveDistanceStrafe(.4, -10);
                                    //turnRightDistance(.5, 13);
                                    //sleep(1000);
                                    //if(recognition.getLabel().equals("Stone"))
                                    //{
                                   //     turnLeftDistance(.5, 13);
                                   // }
                                   // else
                                    //{
                                    //    turnLeftDistance(.5, 36);
                                    //    moveDistance(.5, -60);
                                    //}
                                    sleep(1500);
                                    //index = 4;
                                    count++;


                                }

                                else
                                {
                                    telemetry.addLine("Nothing Detected");
                                    turnLeftDistance(.5, 1, 300);
                                }
                                //telemetry.addLine("LABEL = " + label);

                            }
                        }
                        telemetry.update();
                        sleep(1500);
                    }
                }

            }
//            while(!detected)
//            {
//                if(detected)
//                {
//
//                    if (stone)
//                    {
//                        moveDistanceStrafe(.5, -10);
//                    }
//                    if (skystone)
//                    {
//                        moveDistance(.5, -10);
//                    }
//
//
//                     telemetry.addLine("DETECTED");
//
//                }
//
//            }

//                moveDistance(.4, -30);
//                leftServo.setPosition(.58);
//                rightServo.setPosition(.3);
//                moveDistance(.4, 5);
//                turnLeftDistance(.4, -23);
//                moveDistance(.4, -35);
//                leftServo.setPosition(0);
//                rightServo.setPosition(0);
//                moveDistance(.4,10);

//            moveDistance(.4, -10);
//            leftServo.setPosition(.58);
//            rightServo.setPosition(.3);
//            moveDistance(.4, 3);
//            turnLeftDistance(.4, -23);
//            moveDistance(.4, -10);
//            leftServo.setPosition(0);
//            rightServo.setPosition(0);
//            moveDistance(.4,4);
//



        }

        if (tfod != null)
        {
            tfod.shutdown();
        }
    }
    public void move(double power)
    {


        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);



    }
    public void stopRobot()
    {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
    public void turnLeft(double power)
    {

        leftFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
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

    public void turnRight(double power)
    {
        turnLeft (-power);
    }
    public void moveDistance(double power, double distance)
    {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int)(distance * COUNTS_PER_INCH );


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

    public void turnLeftDistance (double power, double distance, int sleep)
    {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int)(distance * COUNTS_PER_INCH );

        leftFrontMotor.setTargetPosition(-amountToMove);
        leftRearMotor.setTargetPosition(-amountToMove);
        rightFrontMotor.setTargetPosition(amountToMove);
        rightRearMotor.setTargetPosition(amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnLeft(power);


       // while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())
        //{


        //}
        sleep(sleep);

        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRightDistance (double power, double distance, int sleep)
    {
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightRearMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        int amountToMove = (int)(distance * COUNTS_PER_INCH );

        leftFrontMotor.setTargetPosition(amountToMove);
        leftRearMotor.setTargetPosition(amountToMove);
        rightFrontMotor.setTargetPosition(-amountToMove);
        rightRearMotor.setTargetPosition(-amountToMove);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnRight(power);


        //while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())
        //{


        //}
        sleep(sleep);
        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveDistanceStrafe(double power, double distance, int sleep)
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


        //while (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())
        //{


       // }
        sleep(sleep);
        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initVuforia()
    {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.80;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
