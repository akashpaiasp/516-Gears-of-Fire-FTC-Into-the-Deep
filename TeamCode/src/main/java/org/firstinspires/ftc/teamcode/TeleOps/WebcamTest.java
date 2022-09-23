package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp
public class WebcamTest extends LinearOpMode
{
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View:
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Specify the image processing pipeline we wish to use on a frame from the camera
        webcam.setPipeline(new SamplePipeline());


        // Open the connection to the camera device
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

                //This will be called if the camera could not be opened

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.addData("Type", pipeline.getType());
            telemetry.update();

            // Send some stats to the telemetry
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            //The "if" statement below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
            if(gamepad1.a)
            {
                /*\
                 DO NOT DELETE
                 ~~~~~~~~~~~~~~~~~~~~~
                 ☆ Calling stopStreaming() will indeed stop the stream of images
                   from the camera and stop calling your vision pipeline

                 ☆ If the reason you wish to stop the stream early is to switch use of the camera
                   over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                   (commented out below), because according to the Android Camera API documentation:

                          "Your application should only have one Camera object active at a time for
                           a particular hardware camera."

                 ☆ Calling closeCameraDevice() will internally call stopStreaming() if applicable, but it doesn't hurt to
                   call it anyway, if for no other reason than clarity.

                 ☆ If you are stopping the camera stream temporarily, it is recommended to NOT call closeCameraDevice() as
                   you will then need to re-open it the next time you wish to activate your vision pipeline
                ~~~~~~~~~~~~~~~~~~~~~
                \*/

                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            // Throttle ourselves to 10Hz (100 ms) loop
            // By default, telemetry is only sent to the DS at 4Hz
            sleep(100);
        }
    }

    /*\
    DO NOT DELETE
    ~~~~~~~~~~~~~~~~~~~~~
     ☆ Below is an example image processing pipeline to be run upon receipt of each frame from the camera.

     ☆ A new camera frame will not come in while you're still processing a previous one.
       In other words, the processFrame() method will never be called multiple times simultaneously.

     ☆ However, the rendering of your processed image to the viewport is done in parallel to the
       frame worker thread. That is, the amount of time it takes to render the image to the
       viewport does NOT impact the amount of frames per second that your pipeline can process.
    ~~~~~~~~~~~~~~~~~~~~~
    \*/

    public static class SamplePipeline extends OpenCvPipeline
    {
        private static final Scalar BLUE = new Scalar(0,0,255);
        Point topLeft = new Point(50, 50);
        Point bottomRight = new Point(100, 100);
        private static final int THRESHOLD = 107; // We have found 107 to work well

        Mat region; // The region we will average
        Mat YCrCb; // The entire camera view
        Mat Cb = new Mat(); // Cb channel for the camera view
        private volatile int average; // the average of our channel;
        private volatile TYPE type;

        private void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }
        @Override
        public void init(Mat mat){
            inputToCb(mat);
            region = Cb.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            average = (int)Core.mean(region).val[0];
            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);

            if(average > THRESHOLD)
            {
                type = TYPE.BALL;
            }
            else
            {
                type = TYPE.CUBE;
            }

            return input;
        }


        public TYPE getType()
        {
            return type;
        }

        public int getAverage()
        {
            return average;
        }

        enum TYPE
        {
            BALL, CUBE
        }

    }
}