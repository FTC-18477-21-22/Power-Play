package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutoRightFar extends LinearOpMode {

    //static final double COUNTS_PER_MOTOR_REV = 28 ;
    //static final double DRIVE_GEAR_REDUCTION = 13.7;
    //static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = 45.2079566;
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();

    private BNO055IMU imu;
    static final double TURN_SPEED = 0.75;
    static final double P_TURN_COEFF_1 = 0.025;
    static final double P_TURN_COEFF_2 = 0.0035;
    static final double HEADING_THRESHOLD = 0.5;
    String signal = "NO_VALUE";
    //imu stuff end

    OpenCvWebcam webcam;
    SignalDetectionPipeline pipeline;
    public enum Signal
    {S1, S2, S3}

    @Override
    public void runOpMode() throws InterruptedException {
        //    signal = "one";

        robot.init(this.hardwareMap);
        robot.Intake.setPower(1);
        robot.Arm.setPosition(0.45);
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap. appContext. getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName. class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SignalDetectionPipeline();
        webcam.setPipeline(pipeline);
        //   webcam.setMillisecondsPermissionTimeout(5000);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new  OpenCvCamera. AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();
        sleep(50);
        Signal sig = pipeline.getAnalysis();
        telemetry.addData("Analysis", sig);
        telemetry.update();

        double distance = 100;
        int counts = (int)(COUNTS_PER_INCH*distance);
        robot.LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*5));
        robot.strafe(0.35, (int)(COUNTS_PER_INCH*30));
        robot.gyroTurn(0.5, 0, 0.01);
        robot.notallwait(0.7, (int)-3700);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*45));
        robot.strafe(0.35, (int)(COUNTS_PER_INCH*-10.5));
        robot.gyroTurn(0.5, 0, 0.01);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*5.5));
        robot.Arm.setPosition(0);
        robot.Intake.setPower(-1);
        sleep(500);
        robot.Intake.setPower(0);
        robot.drive(0.35, (int)-(COUNTS_PER_INCH*5.5));
        robot.notallwait(0.7, (int)0);
        robot.strafe(0.35, (int)-(COUNTS_PER_INCH*13));
        robot.gyroTurn(0.5, 0, 0.01);
        robot.drive(0.35, (int)-(COUNTS_PER_INCH*21));
        // End of robot drop
        switch (sig) {
            case S1:
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*28));
                robot.strafe(0.35, (int)-(COUNTS_PER_INCH*2));
                break;
            case S2:
                /*robot.strafe(0.35, (int)(COUNTS_PER_INCH*-27));
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*2));*/
                break;
            case S3:
                //robot.strafe(0.35, (int)(COUNTS_PER_INCH*-50));
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*-26));
                break;
        }
        while(opModeIsActive()) {
            //empty
        }

        // robot.gyroTurn(TURN_SPEED, 180, P_TURN_COEFF_1);

        // robot.drive(0.50, (int)(45.2079566*100));
        /*sleep(5000);
        robot.gyroTurn(TURN_SPEED, 0, P_TURN_COEFF_2);
        telemetry.addData("turn", "finished");
        telemetry.update();*/

    }

    public static class SignalDetectionPipeline extends OpenCvPipeline
    {
        static final Point TOPLEFT_ANCHOR_POINT = new Point(160,60);
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 80;
        Point region_pointA = new Point(
                TOPLEFT_ANCHOR_POINT.x,
                TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        /*
         * Working variables
         */
        Mat region_Cb;
        Mat region_Ca;
        //   Mat BGR = new Mat();
        Mat LAB = new Mat();
        Mat Cb = new Mat();
        Mat Ca = new Mat();
        int avg_Ca,avg_Cb;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Signal sig = Signal.S1;

        void inputToCb(Mat input)
        {
            //     BGR = Imgcodecs.imdecode(input,Imgcodecs.IMREAD_COLOR);
            //    Core.extractChannel(BGR,Cb,0);
            Imgproc.cvtColor(input,LAB,Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB,Cb,2);
        }
        void inputToCa(Mat input){
            Imgproc.cvtColor(input,LAB,Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB,Ca,1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);   //For Blue alliance
            inputToCa(firstFrame);
            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region_Cb = Cb.submat(new Rect(region_pointA, region_pointB));
            //         region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            //         region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
            region_Ca = Ca.submat(new Rect(region_pointA, region_pointB));

        }

        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            inputToCa(input);
            avg_Cb = (int) Core.mean(region_Cb).val[0];
            avg_Ca = (int) Core.mean(region_Ca).val[0];
            Imgproc.rectangle(
                    input,
                    region_pointA,
                    region_pointB,
                    new Scalar(0, 255, 0), 4);
            if(avg_Cb<120)
            {
                sig = Signal.S1;
            }
            else if (avg_Ca<130)
            {
                sig= Signal.S3;
            }
            else
            {
                sig = Signal.S2;
            }

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }
        public Signal getAnalysis() {return sig;}

    }

}

