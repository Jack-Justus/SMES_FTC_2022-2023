package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
public class WebcamTest extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wbcam"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */

            if(gamepad1.a)
            {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            sleep(100);
        }
    }

    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {

            //TODO: WHITE BALANCING
            //BASICALLY FIND AVERAGE COLOR VAL OF IMAGE AND SUBTRACT/ADD IN ORDER TO BALANCE IT
            //TO A PREDETERMINED IMAGE
            //MAY BE BUILT IN OPENCV FUNCTION, WILL TEST SOON

            int parkSpot = -1;
            int width = input.width(); //width of image
            int height = input.height(); //height of image
            int channels = input.channels(); //color channels, may be obselete since should always be 3

            int offset = 40; //tbd, will be significantly lower once balance implemented

            //counts instances of what may be a cone spot, the largest one will
            int [] parkArray = {0,0,0};

            //goes through image, i corresponds to img height, j to width, finds necessary color vals to indicate where to park
            for (int i = 0; i < height; i+= 5) {
                for (int j = 0; j < width-20; j+= 5) {
                    double[] data = input.get(i,j);
                    double[] forwardData = input.get(i,j+20);
                    //forward data is a few pixels ahead of the current pixel
                    //basically this is important since the cones are half one color (for example red) and half another (for examples blue)
                    //if we have a red pixel and a blue pixel is followed by it, it indicates that the parking cone may be there, this increments the count
                    int currRed = (int) data[0];
                    int currGreen = (int) data[1];
                    int currBlue = (int) data[2];

                    int fwdRed = (int) forwardData[0];
                    int fwdGreen = (int) forwardData[1];
                    int fwdBlue = (int) forwardData[2];

                    //if current is mostly red and forward is mostly blue, it could be the 1st s pot
                    if ((currRed >= 255 - offset && currGreen <= offset && currBlue <= offset) && (fwdRed <= offset && fwdGreen <= offset && fwdBlue >= 255 - offset)) {
                        parkArray[0] += 1;
                    }

                    //if current is mostly white and forward is mostly green, it could be the 2nd spot
                    if ((currRed >= 255 - offset && currGreen >= 255 - offset && currBlue >= 255 - offset) && (fwdRed <= offset && fwdGreen >= 255 && fwdBlue <= offset)) {
                        parkArray[1] += 1;
                    }

                    //if current is mostly black and forward is mostly yellow, it could be the 3rd spot
                    if ((currRed <= offset && currGreen <= offset && currBlue <= offset) && (fwdRed >= 255 - offset&& fwdGreen >= 255 - offset && fwdBlue <= offset)) {
                        parkArray[2] += 1;
                    }
                }
            }

            //the largest array value will be where to park
            if (parkArray[0] >= parkArray[1] && parkArray[0] >= parkArray[2]) {
                parkSpot = 1;
            }
            if (parkArray[1] >= parkArray[0] && parkArray[1] >= parkArray[2]) {
                parkSpot = 2;
            }
            if (parkArray[2] >= parkArray[0] && parkArray[2] >= parkArray[1]) {
                parkSpot = 3;
            }

            telemetry.addData("park spot", parkSpot);
            telemetry.update();
            return input;
        }
    }
}