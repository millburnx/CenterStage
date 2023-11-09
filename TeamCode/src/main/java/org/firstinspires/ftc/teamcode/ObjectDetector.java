package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name="ObjectDetector", group="Auto")
public class ObjectDetector extends LinearOpMode {
    OpenCvWebcam webcam;
    ObjectDetectorPipeline pipeline;
    String section;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ObjectDetectorPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
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

        // create auton class here

        while (opModeIsActive()) {
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

            if(gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }

    class ObjectDetectorPipeline extends OpenCvPipeline {
        boolean viewportPaused;
        int frameCount = 0;
        int latest_x, latest_y;

        public Mat processFrame(Mat input) {
            Mat temp = new Mat();
            Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);
            Mat mask = new Mat();
            Mat mask2 = new Mat();
            Mat res = new Mat();

            Scalar lowerVal = new Scalar(0, 100, 100);
            Scalar upperVal = new Scalar(10, 255, 255);

            Scalar lowerVal2 = new Scalar(100, 100, 100);
            Scalar upperVal2 = new Scalar(255, 255, 255);

            Core.inRange(temp, lowerVal, upperVal, mask);
            Core.inRange(temp, lowerVal2, upperVal2, mask2);
            Mat finalMask = new Mat();
            Core.bitwise_or(mask, mask2, finalMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
            Imgproc.erode(finalMask, finalMask, kernel);
            Imgproc.dilate(finalMask, finalMask, kernel);

            Core.bitwise_and(input, input, res, finalMask);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(finalMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = -1;
            int maxAreaIdx = -1;

            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    maxAreaIdx = i;
                }
            }

            if (maxAreaIdx != -1) {
                Moments mu = Imgproc.moments(contours.get(maxAreaIdx));
                int centerX = (int) (mu.get_m10() / mu.get_m00());
                int centerY = (int) (mu.get_m01() / mu.get_m00());
                this.latest_x = centerX;
                this.latest_y = centerY;
                Imgproc.circle(res, new Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            }

            telemetry.addLine("x: " + this.latest_x);
            telemetry.addLine("y: " + this.latest_y);

            int region = this.latest_x * 3 / res.width();
            if (region == 2) {
                section = "left";
            } else if (region == 0) {
                section = "right";
            } else {
                section = "middle";
            }

            telemetry.addLine("The beacon in in the: " + section);
            telemetry.update();

            return res;
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
