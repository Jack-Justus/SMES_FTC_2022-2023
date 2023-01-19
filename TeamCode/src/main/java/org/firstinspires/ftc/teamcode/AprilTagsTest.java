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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.openftc.apriltag.*;
import org.openftc.easyopencv.*;

import java.util.ArrayList;

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
@TeleOp
public class AprilTagsTest extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
    private DcMotor vertLinearSlide = null;
    SampleMecanumDrive drive = null;
    private CRServo claw = null;

    // hor slide (no claw)

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //tags from tag family thing
    int PARK_LEFT = 3;
    int PARK_MIDDLE = 15;
    int PARK_RIGHT = 17;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {

            //TODO: put roadrunner stuff here up until it detects the april tag here, ie going forward and dropping cone on pole


            //detection stuff happens, skip to line 186
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wbcam"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.setMsTransmissionInterval(50);

            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == PARK_LEFT || tag.id == PARK_MIDDLE || tag.id == PARK_RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see any parking tags");

                        if (tagOfInterest == null) {
                            telemetry.addLine("Don't see any parking tags");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tags");

                    if (tagOfInterest == null) {
                        telemetry.addLine("tags not seen");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
                telemetry.update();
                sleep(20);
            }

            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            //this part will actually park for us
            //you can ignore everything above this if you want
            if (tagOfInterest.id == PARK_LEFT) {
                //TODO: code for parking in the left square
            }
            if (tagOfInterest.id == PARK_MIDDLE || tagOfInterest == null) {
                //TODO: code for parking in the middle square
            }
            if (tagOfInterest.id == PARK_RIGHT) {
                //TODO: code for parking in the right square
            }


            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        }

        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void initializeHardware() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
//        drive = new SampleMecanumDrive(hardwareMap);
//        vertLinearSlide = hardwareMap.get(DcMotor.class, "vertSlide");
////        horLinearSlide = hardwareMap.get(DcMotor.class, "horSlide");
//        claw = hardwareMap.get(CRServo.class, "claw");
//
//        // init slides
//        vertLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        vertLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        vertLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        horLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Setting the motor encoder position to zero
//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensuring the motors get the instructions
        sleep(100);

        // This makes sure the motors are not using encoders (we don't use them)
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

    }
}