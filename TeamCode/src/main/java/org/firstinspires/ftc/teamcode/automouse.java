package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Autonomous", group = "Linear Opmode")
public class automouse extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Left and Right Drive Motor Objects
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    //will be figured out in encoder test
    private final float DIST_2_ENCODER = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //initialize motor things
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");

        while (opModeIsActive()) {


        }

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

        leftFrontDrive.setPower(Range.clip(x + y, -1.0, 1.0));
        leftBackDrive.setPower(Range.clip(y - x, -1.0, 1.0));

        rightFrontDrive.setPower(-Range.clip(y - x, -1.0, 1.0));
        rightBackDrive.setPower(Range.clip(x + y, -1.0, 1.0));

    }

    public void moveForward(int seconds) {

        if (seconds == 0)
            seconds = 1;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000) && opModeIsActive())
            move(0);
    }

    public void moveRight(int seconds) {

        if (seconds == 0)
            seconds = 1;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(1);
    }

    public void moveBack(int seconds) {

        if (seconds == 0)
            seconds = 1;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(2);
    }

    public void moveLeft(int seconds) {

        if (seconds == 0)
            seconds = 1;

        double baseline = runtime.milliseconds();

        // Running until the amount of time has elapsed
        while (runtime.milliseconds() < baseline + (seconds * 1000))
            move(3);
    }
}
