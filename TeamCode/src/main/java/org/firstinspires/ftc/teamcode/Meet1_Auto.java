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

import static org.firstinspires.ftc.teamcode.AutoData.encoderInchesToTicks;
import static org.firstinspires.ftc.teamcode.AutoData.encoderTicksToInches;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//drive straight forward
// (line up robot touching back wall at an angle so that driving straight forward corresponds with tallest pole)
// then, lift arm, drop off pre-load cone
@Autonomous
public class Meet1_Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Left and Right Drive Motor Objects
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;

    //Slide and Claw Objects
    private DcMotor vertLinearSlide = null;
    private CRServo claw = null;

    private int autoPhase = 0;

    //encoder
    StandardTrackingWheelLocalizer encoders;

    @Override
    public void runOpMode() {

        // Method to assign and initialize the hardware
        initializeHardware();
        setMotorsBreakMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            /*
            Our Autonomous is broken into phases.
            This is so that we don't have the motors receiving contradictory commands.
            Once one phase of instructions is complete, we will move to the next.
             */

            switch (autoPhase) {

                case 0:
                    // Starting by moving and raising the lift
                    if (moveLift(1000) && move(10, 0)) {
                        sleep(1000);
                        autoPhase = 1;
                    }
                    break;

                case 1:
                    // Computer vision
                    switch (checkConeState()) {

                        case 0:
                            break;
                        case 1:
                            break;
                        case 2:
                            break;
                        default:
                            // TODO: Thoughts on what we should do if the camera can't see anything?


                            break;
                    }
                    break;


            }
        }
    }

    private int checkConeState() {
        // Andrew this function is for your computer vision

        return 0;
    }

    private boolean moveLift(int targetTicks) {

        // Moving the lift to the target position

        // Constants
        double liftTicks = vertLinearSlide.getCurrentPosition();
        final double LIFT_IDLE_POWER = 0.3;
        final double LIFT_MID_POWER = -0.3;
        final double LIFT_MAX_POWER = 1.0;


        // Checking to see if the lift is at the position with a tolerance of ±50 ticks
        if (Math.abs(liftTicks - targetTicks) < 50) {
            vertLinearSlide.setPower(LIFT_IDLE_POWER);
            return true;
        }


        double distToTarget = -(liftTicks - targetTicks);

        double calcPower = Range.clip(distToTarget / 1000, LIFT_MID_POWER, LIFT_MAX_POWER);

        vertLinearSlide.setPower(calcPower);

        return false;
    }


    public boolean move(double inches, int dir) {

        /* DIR KEY
        0 = forward
        1 = right
        2 = back
        3 = left
         */

        // We measure encoder distance based on the average of the left and right encoder
        double encoderPos = (encoders.getWheelPositions().get(0) + encoders.getWheelPositions().get(1)) / 2;
        double ticks = encoderInchesToTicks(inches);

        // Moving forward if we are not at the destination
        if (encoderPos < ticks)
            move(dir);
        else
            return true;
        return false;
    }

    private void move(int dir) {

        /*
        0 = forward
        1 = right
        2 = back
        3 = left
         */

        int x;
        int y;


        switch (dir) {

            case 0:
                y = -1;
                x = 0;
                break;
            case 1:
                y = 0;
                x = -1;
                break;
            case 2:
                y = 1;
                x = 0;
                break;
            case 3:
                y = 0;
                x = 1;
                break;
            default:
                x = 0;
                y = 0;
                break;
        }

        if (opModeIsActive()) {
            leftFrontDrive.setPower(Range.clip(x + y, -1.0, 1.0));
            leftBackDrive.setPower(Range.clip(y - x, -1.0, 1.0));

            rightFrontDrive.setPower(Range.clip(y - x, -1.0, 1.0));
            rightBackDrive.setPower(Range.clip(x + y, -1.0, 1.0));
        }
    }

    private void closeClaw() {
        claw.setPower(1);
    }

    private void openClaw() {
        claw.setPower(-1);
    }

    private void initializeHardware() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        vertLinearSlide = hardwareMap.get(DcMotor.class, "vertSlide");
//        horLinearSlide = hardwareMap.get(DcMotor.class, "horSlide");
        claw = hardwareMap.get(CRServo.class, "claw");

        // init slides
        vertLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        horLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Setting the motor encoder position to zero
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensuring the motors get the instructions
        sleep(100);

        // This makes sure the motors are moving at the same speed
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Encoders
        encoders = new StandardTrackingWheelLocalizer(hardwareMap);

        setMotorsBreakMode();
    }

    public void setMotorsBreakMode() {
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsFloatMode() {
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}

// Old stuff

//            //close claw to tighten around pre-load cone
//            claw.setPosition(1);
//            //give the claw closing time before jerking forward
//            sleep(100);
//            leftFrontDrive.setPower(1);
//            leftBackDrive.setPower(1);
//            rightBackDrive.setPower(1);
//            rightFrontDrive.setPower(1);
//            // once this loop completes, the slide should be at the top
//            // (yes, the cone is dangling in the air)
//            // yes, it has a risk of falling
//            // however, our lack of encoder/time/autonmous in general accuracy,
//            // the less driving adjustments we have to make the better
//            while (linearSlide.getCurrentPosition() <= MAX_SLIDE_TICKS) {
//                linearSlide.setPower(-1);
//            }
//            // COMMENT OUT NOT-USED OPTIONS
//            //the time to be driving forward if not using encoders:
//            sleep(500);
//            // if we are using encoders:
//            if (STOP_DIST_TICKS >= encoder.getCurrentPosition()) {
//                leftFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                // resetting to 0 so that we can back up
//                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            // drop the cone
//            claw.setPosition(0.7);
//            // back-up
//            leftFrontDrive.setPower(-1);
//            leftBackDrive.setPower(-1);
//            rightBackDrive.setPower(-1);
//            rightFrontDrive.setPower(-1);
//            // TIME
//            sleep(500);
//            // OR TICKS
//            if (BACK_UP_TICKS >= encoder.getCurrentPosition()) {
//                leftFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                // resetting to 0 so that we can back up
//                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            //put the slide down after backing up
//            while (linearSlide.getCurrentPosition() <= MAX_SLIDE_TICKS) {
//                linearSlide.setPower(1);
//            }

//    private void raiseLift(int ticks) {
//
//        // Going up smoothly
//
//        // If the linear slide has hit the top then the upperBoundHit becomes true
//        // We can only move the linear slide up if upperBoundHit is false
//        // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
//        if (vertLinearSlide.getCurrentPosition() >= ticks)
//            upperBoundHit = true;
//        else if (vertLinearSlide.getCurrentPosition() < ticks - 100)
//            upperBoundHit = false;
//
//        // If the current position is valid, we move the motor upwards
//        // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
//        if ((vertLinearSlide.getCurrentPosition() < ticks - 150) && (!upperBoundHit))
//            vertLinearSlide.setPower(1);
//        else if ((vertLinearSlide.getCurrentPosition() < ticks && (!upperBoundHit)))
//            vertLinearSlide.setPower(0.4);
//        else if (vertLinearSlide.getCurrentPosition() > 100)
//            // To prevent downward drift
//            vertLinearSlide.setPower(0.3);
//
//    }