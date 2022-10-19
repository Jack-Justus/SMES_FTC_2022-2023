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

            int parkSpot = -1;
            int width = input.width(); //width of image
            int height = input.height(); //height of image
            int channels = input.channels(); //color channels, may be obselete since should always be 3

            int prevR = -1;
            int prevG = -1;
            int prevB = -1;
            int offset = 100; //tbd, will be significantly lower once balance impleme


            //goes through image, finds necessary color vals to indicate where to park
            //park in spot 1 - neon green (rgb: ~50 ~255 ~20) and bright pink (rgb: ~255 ~0 ~125)
            //spot 2 - bright yellow (rgb: ~255, ~255, ~0) and full blue (rgb: ~0 ~0 ~255)
            //spot 3 - red (rgb:~255 ~0 ~0 ) and full green (rgb: ~0 ~255 ~0)
            for (int i = 0; i < height; i+= 5) {
                for (int j = 0; j < width; j+= 5) {
                    double[] data = input.get(i,j);
                    int currRed = (int) data[0];
                    int currGreen = (int) data[1];
                    int currBlue = (int) data[2];
                    if (currRed >= 255 - offset && currGreen <= 0 + offset  && currBlue <= 0 + offset) {
                        parkSpot = 0;
                    }

                    if (currRed <= 0 + offset && currGreen >= 255 - offset  && currBlue <= 0 + offset) {
                        parkSpot = 1;
                    }

                    if (currRed <= 0 + offset && currGreen <= 0 + offset  && currBlue >= 255 - offset) {
                        parkSpot = 2;
                    }
                    prevR = currRed;
                    prevG = currGreen;
                    prevB = currBlue;
                }
            }
            telemetry.addData("park spot", parkSpot);
            telemetry.update();
            return input;
        }
    }
}