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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Normal Drive Mode", group = "Linear Opmode")
public class MechanumDriveMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Left and Right Drive Motor Objects
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;

    private boolean slowModeActive = false;

    private final int DEFAULT_MOVE_TIME = 1;

    @Override
    public void runOpMode() {

        // HEADS UP
        // THE MOTOR NAMES REFER TO THE ROBOT FROM BEHIND
        // FOR EXAMPLE - THE FRONT LEFT MOTOR IS ACTUALLY CALLED THE BACK RIGHT MOTOR IN CODE


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Method to assign and initialize the hardware
        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        float x;
        float y;
        float rot;
        float rotSpeed;

        double lfp;
        double lbp;
        double rfp;
        double rbp;

        double speedModifier;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // The Letter buttons override the rest of this function
            if (gamepad1.y)
                moveForward(0);
            else if (gamepad1.a)
                moveBack(0);
            if (gamepad1.x)
                moveLeft(0);
            else if (gamepad1.b)
                moveRight(0);


            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

            rot = -gamepad1.right_stick_x;

            // Range.clip(value, -1.0, 1.0);

            // Drive Code
            lfp = Range.clip(x + y, -1.0, 1.0);
            lbp = Range.clip(y - x, -1.0, 1.0);

            rfp = Range.clip(y - x, -1.0, 1.0);
            rbp = Range.clip(x + y, -1.0, 1.0);

            // Fast rotation
            if (gamepad1.right_stick_button)
                rotSpeed = 1;
            else
                rotSpeed = 2;

            // Slow mode
            if (gamepad1.right_bumper)
                slowModeActive = true;
            else
                slowModeActive = false;

            if (slowModeActive) {
                speedModifier = .5;
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
            rightBackDrive.setPower((-rbp) * speedModifier);
            rightFrontDrive.setPower((rfp) * speedModifier);
            leftFrontDrive.setPower((-lfp) * speedModifier);
            leftBackDrive.setPower((-lbp) * speedModifier);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "lf + rb (%.2f), lb + rf (%.2f)", lfp, rfp);
            telemetry.update();
        }
    }

    private void initializeHardware() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");

        // Setting the motor encoder position to zero
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensuring the motors get the instructions
        sleep(100);

        // This makes sure the motors are moving at the same speed
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    // These functions are to prototype and test autonomous

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

        leftFrontDrive.setPower(Range.clip(x + y, -1.0, 1.0));
        leftBackDrive.setPower(Range.clip(y - x, -1.0, 1.0));

        rightFrontDrive.setPower(-Range.clip(y - x, -1.0, 1.0));
        rightBackDrive.setPower(Range.clip(x + y, -1.0, 1.0));

    }

    // These four functions can probably be compressed so if u find a way to do that be my guest

    public void moveForward(int seconds) {

        if (seconds == 0)
            seconds = DEFAULT_MOVE_TIME;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000) && opModeIsActive())
            move(0);
    }

    public void moveRight(int seconds) {

        if (seconds == 0)
            seconds = DEFAULT_MOVE_TIME;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(1);
    }

    public void moveBack(int seconds) {

        if (seconds == 0)
            seconds = DEFAULT_MOVE_TIME;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(2);
    }

    public void moveLeft(int seconds) {

        if (seconds == 0)
            seconds = DEFAULT_MOVE_TIME;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(3);
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
