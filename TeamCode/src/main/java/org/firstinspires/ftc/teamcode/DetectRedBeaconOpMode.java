package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Red Beacon Detector", group="Auto")
public class DetectRedBeaconOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    RedBeaconDetector pipeline;


    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new RedBeaconDetector();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
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
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("Error lmao");
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }

    class RedBeaconDetector extends OpenCvPipeline
    {
        boolean viewportPaused;
        int frameCount = 0;

        public Mat processFrame(Mat input) {
            // Convert the input Mat to the BGR color space (if it's not already)
            Mat bgrMat = new Mat();
            Imgproc.cvtColor(input, bgrMat, Imgproc.COLOR_RGBA2BGR);

            // Define the lower and upper bounds for the red color in BGR format
            Scalar lowerRed = new Scalar(0, 0, 100); // Adjust these values to match your specific red color
            Scalar upperRed = new Scalar(100, 100, 255);

            // Split the frame into three equal vertical strips
            int width = bgrMat.width();
            int height = bgrMat.height();
            int stripWidth = width / 3;

            int[] redPixelCounts = new int[3];

            for (int i = 0; i < 3; i++) {
                // Define the region of interest (ROI) for the current strip
                Rect roi = new Rect(i * stripWidth, 0, stripWidth, height);
                Mat redStrip = new Mat(bgrMat, roi);

                // Create a mask that isolates the red pixels within the specified range
                Mat redMask = new Mat();
                Core.inRange(redStrip, lowerRed, upperRed, redMask);

                // Find the total number of red pixels in the mask
                redPixelCounts[i] = Core.countNonZero(redMask);

                // Release Mats to prevent memory leaks
                redStrip.release();
                redMask.release();
            }

            // Find the section with the most red pixels
            int maxRedPixelsIndex = 0;
            for (int i = 1; i < 3; i++) {
                if (redPixelCounts[i] > redPixelCounts[maxRedPixelsIndex]) {
                    maxRedPixelsIndex = i;
                }
            }

            // Release the bgrMat
            bgrMat.release();

            // You can use maxRedPixelsIndex to identify the section with the most red pixels
            telemetry.addLine("Section with the most red pixels: " + maxRedPixelsIndex);

            return input; // Return the original frame for display (you can modify this if needed)
        }

        /*
        @Override
        public Mat processFrame(Mat input) {
            // Make a copy of the input Mat to avoid modifying the original
            Mat frame = new Mat();
            input.copyTo(frame);

            // Convert the frame to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            // Define the lower and upper bounds of the red color in HSV
            Scalar lowerRed = new Scalar(0, 100, 100);
            Scalar upperRed = new Scalar(10, 255, 255);

            // Create a binary mask for the red color
            Mat mask = new Mat();
            Core.inRange(hsv, lowerRed, upperRed, mask);

            // Find contours in the mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Initialize variables for screen thirds and largest contour center
            int screenThirds = -1; // -1 for not found, 0 for left, 1 for middle, 2 for right
            Point largestContourCenter = new Point(0, 0);

            // Find the largest contour
            double maxArea = 0;
            int maxAreaContourIdx = -1;
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    maxAreaContourIdx = i;
                }
            }

            if (maxAreaContourIdx != -1) {
                MatOfPoint largestContour = contours.get(maxAreaContourIdx);
                // Calculate the center of mass of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;

                // Update the largest contour center
                largestContourCenter = new Point(cx, cy);

                // Determine which third of the screen the object is in
                int third = (int) (cx * 3 / frame.width());

                if (third == 0) {
                    screenThirds = 0; // Left
                } else if (third == 1) {
                    screenThirds = 1; // Middle
                } else if (third == 2) {
                    screenThirds = 2; // Right
                }
            }

            // Release Mats to avoid memory leaks
            frame.release();
            hsv.release();
            mask.release();

            // Display the center and screen third where the red item is located
            telemetry.addLine("Center of the largest contour: X=" + largestContourCenter.x + ", Y=" + largestContourCenter.y);
            telemetry.addLine("Red item is in the " + (screenThirds == -1 ? "unknown" : (screenThirds == 0 ? "left" : (screenThirds == 1 ? "middle" : "right"))) + " third.");

            return input;
        }
        */

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}
