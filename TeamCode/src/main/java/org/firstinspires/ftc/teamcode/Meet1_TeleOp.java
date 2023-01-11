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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

// WHAT IS WHAT
// gamepad.2 is "weapons" - claw (x- close, y- open) - slide (a- up, b- down)
// gamepad.1 is driving - motors dual-stick drive, right bumper for slow mode


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Normal Drive Mode", group = "Linear Opmode")
public class Meet1_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Left and Right Drive Motor Objects
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;

    // Odometers
//    private DcMotor encoderLeft;
//    private DcMotor encoderRight;
//    private DcMotor encoderAux;
//
//    // Encoder Vars and stuff
//    final static double L = 22.86;      // Distance between encoder 1 and 2 in cm
//    final static double B = 0;          // Distance between the midpoint of encoder 1 and 2 and encoder 3
//    final static double R = 9.8;        // Wheel radius in cm
//    final static double N = 8192;       // Encoder ticks per revolution, REV encoder
//    final static double cm_per_tick = 2.0 * Math.PI * R / N;
//
//    // Odometry math to keep track between updates
//    public int currentRightPosition = 0;
//    public int currentLeftPositions = 0;
//    public int currentAuxPosition = 0;
//
//    private int oldRightPosition = 0;
//    private int oldLeftPosition = 0;
//    private int oldAuxPosition = 0;

    /************
     * Odometry
     * Notes:
     * n1, n2, n3 are encoder values for the left, right and back (aux) omni-wheels
     * dn1, dn2, dn3 are the differences of encoder values between two reads
     * dx, dy, dtheta describe the robot movement between two reads (in robot coordinates)
     * X, Y, Theta are the coordinates on the field and the heading of the robot
     ************************************/

    // XyhVector is a tuple (x,y,h) where h is the heading of the robot
    // Starting at (0,0) pos with 0 rotation. Our auto will run off of this coordinate system
//    public XyhVector START_POS = new XyhVector(0, 0, Math.toRadians(0));
//    public XyhVector pos = START_POS;

    // vert Slide and Claw Objects
    private DcMotor vertLinearSlide = null;
    private Servo claw = null;

    // hor slide (no claw)
    private DcMotor horLinearSlide = null;


    // Used for vert linear slide
    private boolean upperBoundHit = false;
    private boolean lowerBoundHit = false;
    // MAX_TICKS is the value at the top (don't raise up more than this)
    // MIN_TICKS is the value at the bottom (don't wind up more than this)
    final int MAX_TICKS = 3250;
    final int MIN_TICKS = 0;

    @Override
    public void runOpMode() {

        // HEADS UP
        // THE MOTOR NAMES REFER TO THE ROBOT FROM BEHIND
        // FOR EXAMPLE - THE FRONT LEFT MOTOR IS ACTUALLY CALLED THE BACK RIGHT MOTOR IN CODE

        // Method to assign and initialize the hardware
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Driving robot is handled on gamepad1
            driveCode();

            // Claw is controlled by gamepad2
            controlClaw();

            // The vertical slide is controlled by gamepad2
            controlVertSlide();

            // The Horizontal slide is controlled by gamepad1
            controlHorizontalSlide();

            // No driver controls this
            //odometry();


//            telemetry.addData("Slide Encoder", "Value (%.2f)", linearSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    private void controlHorizontalSlide() {

        /***********************
         * HORIZONTAL SLIDE HARDWARE MAP
         *
         *      BUTTON A:
         * SLIDE IN
         *
         *      BUTTON B:
         * SLIDE OUT
         *
         * WE GOING YOLO NO ENCODER LIMITS JUST DON'T BREAK THE ROBOT
         *
         *********************/

        Gamepad gp = gamepad1;

        if (gp.a)
            horLinearSlide.setPower(1);
        else if (gp.b)
            horLinearSlide.setPower(-1);
        else
            horLinearSlide.setPower(0);
    }

    private void driveCode() {

        /**********************
         * DRIVE MAP
         *
         *      LEFT STICK:
         * FWD, BWD, L, AND R
         *
         *      RIGHT STICK:
         * ROTATE L AND R
         *
         *      RIGHT STICK (PRESS):
         * ROTATE FAST (HOLD)
         *
         *      RIGHT BUMPER:
         * SLOWMODE
         *
         *********************/

        // Drive vars
        double lfp, lbp, rfp, rbp;
        float x, y, rot, rotSpeed;
        boolean slowModeActive;
        double speedModifier;

        // Driving is handled on gamepad1
        Gamepad gp = gamepad1;

        x = -gp.left_stick_x;
        y = gp.left_stick_y;

//        x = (float) ((runtime.milliseconds() % 5000) / 5000);
//        y = (float) (((runtime.milliseconds() + 2500) % 5000) / 5000);

        rot = -gp.right_stick_x;

        double maxSpeed = 0.7;

        // Drive Code
        lfp = Range.clip(x + y, -maxSpeed, maxSpeed);
        lbp = Range.clip(y - x, -maxSpeed, maxSpeed);

        rfp = Range.clip(y - x, -maxSpeed, maxSpeed);
        rbp = Range.clip(x + y, -maxSpeed, maxSpeed);

        // Fast rotation
        if (gp.right_stick_button)
            rotSpeed = 1;
        else
            rotSpeed = 2;

        // Slow mode
        if (gp.right_bumper)
            slowModeActive = true;
        else
            slowModeActive = false;

        if (slowModeActive) {
            speedModifier = .4;
            setMotorsBreakMode();
        } else {
            speedModifier = 1;
            setMotorsFloatMode();
        }

        // Rotational offset code factoring in precalculated drive code
        lfp = Range.clip(lfp - rot / rotSpeed, -1.0, 1.0);
        lbp = Range.clip(lbp - rot / rotSpeed, -1.0, 1.0);

        rfp = Range.clip(rfp + rot / rotSpeed, -1.0, 1.0);
        rbp = Range.clip(rbp + rot / rotSpeed, -1.0, 1.0);

        // Send calculated power to wheels
        rightBackDrive.setPower((rbp) * speedModifier);
        rightFrontDrive.setPower((rfp) * speedModifier);
        leftFrontDrive.setPower((-lfp) * speedModifier);
        leftBackDrive.setPower((-lbp) * speedModifier);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Power", "lf (%.2f), rb (%.2f), lb (%.2f), rf (%.2f)", lfp, rbp, lbp, rfp);
        telemetry.addData("Speed Modifier", "Master (%.2f), Rotational (%.2f)", speedModifier, rotSpeed);
        telemetry.addData("Slide encoder value: ", vertLinearSlide.getCurrentPosition());
        telemetry.update();


    }

    public void odometry() {

        // Updating these vars
//        oldRightPosition = currentRightPosition;
//        oldLeftPosition = currentLeftPositions;
//        oldAuxPosition = currentAuxPosition;

        // Updating the current encoders pos
//        currentRightPosition = -encoderRight.getCurrentPosition();
//        currentLeftPositions = -encoderLeft.getCurrentPosition();
//        currentAuxPosition = encoderAux.getCurrentPosition();

//        int dn1 = currentLeftPositions - oldLeftPosition;
//        int dn2 = currentRightPosition - oldRightPosition;
//        int dn3 = currentAuxPosition - oldAuxPosition;

        // The robot has moved and turned a tiny bit between two measurements:
//        double dtheta = cm_per_tick * (dn2 - dn1) / L;
//        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
//        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);
//
//        // Small movement of the robot gets added to the field coordinate system:
//        double theta = pos.h + (dtheta / 2.0);
//        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
//        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
//        pos.h += dtheta;

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
        horLinearSlide = hardwareMap.get(DcMotor.class, "horSlide");
        claw = hardwareMap.get(Servo.class, "claw");

        // init slides
        vertLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        // Setting up the encoders
//        encoderLeft = hardwareMap.get(DcMotor.class, "ENCODER Left");
//        encoderRight = hardwareMap.get(DcMotor.class, "ENCODER Right");
//        encoderAux = hardwareMap.get(DcMotor.class, "ENCODER Aux");

    }

    private void controlVertSlide() {

        /**********************
         *  VERT SLIDE HARDWARE MAP
         *
         *      BUTTON A:
         *  SLIDE UP
         *
         *      BUTTON B:
         *  SLIDE DOWN
         *
         *      dpad UP:
         *  EMERGENCY (ignore limits) UP
         *
         *      dpad DOWN:
         *  EMERGENCY DOWN
         *
         *      dpad LEFT:
         *  SET ENCODER TO 0
         *
         *********************/

        Gamepad gp = gamepad2;

        if (gp.a) {
            // Going up

            // If the linear slide has hit the top then the upperBoundHit becomes true
            // We can only move the linear slide up if upperBoundHit is false
            // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
            if (vertLinearSlide.getCurrentPosition() >= MAX_TICKS)
                upperBoundHit = true;
            else if (vertLinearSlide.getCurrentPosition() < MAX_TICKS - 100)
                upperBoundHit = false;

            // If the current position is valid, we move the motor upwards
            // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
            if ((vertLinearSlide.getCurrentPosition() < MAX_TICKS - 150) && (!upperBoundHit))
                vertLinearSlide.setPower(1);
            else if ((vertLinearSlide.getCurrentPosition() < MAX_TICKS && (!upperBoundHit)))
                vertLinearSlide.setPower(0.3);
            else
                vertLinearSlide.setPower(0.1);

        } else if (gp.b) {
            // Going downa

            // If the linear slide has hit the top then the upperBoundHit becomes true
            // We can only move the linear slide up if upperBoundHit is false
            // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
            if (vertLinearSlide.getCurrentPosition() <= MIN_TICKS + 25)
                lowerBoundHit = true;
            else if (vertLinearSlide.getCurrentPosition() > MIN_TICKS)
                lowerBoundHit = false;


            // If the current position is valid, we move the motor upwards
            // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
            if ((vertLinearSlide.getCurrentPosition() > MIN_TICKS) && (!lowerBoundHit))

                // It needs to move slower compared to how close it is to the bottom
                if (vertLinearSlide.getCurrentPosition() > 750)
                    vertLinearSlide.setPower(-1);
                else if (vertLinearSlide.getCurrentPosition() > 275)
                    vertLinearSlide.setPower(-.4);
                else
                    vertLinearSlide.setPower(-.2);
            else
                vertLinearSlide.setPower(0);
        }
        // The following code is used for emergencies when the encoder has drifted or we need to remove limits for the motor
        else if (gp.dpad_up)
            vertLinearSlide.setPower(0.3);
        else if (gp.dpad_down)
            vertLinearSlide.setPower(-0.3);
        else if (gp.dpad_left) {
            vertLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            vertLinearSlide.setPower(0);
        }
    }

    public void controlClaw() {

        Gamepad gp = gamepad2;

        /**********************
         * CLAW MAP
         *
         *      BUTTON X:
         * CLOSE CLAW
         *
         *      BUTTON Y:
         * OPEN CLAW
         *
         *********************/

        double CLAW_CLOSED = 1;
        double CLAW_OPEN = 0;

        if (gp.x)
            claw.setPosition(CLAW_CLOSED);
        else if (gp.y)
            claw.setPosition(CLAW_OPEN);


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
