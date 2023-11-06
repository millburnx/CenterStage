package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

@Autonomous(name="Detect Red Beacon", group="Auto")
public class DetectRedBeaconOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    DetectBeaconPipeline pipeline;
    String section;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new DetectBeaconPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error lmao");
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        boolean autonRun = false;

        while (opModeIsActive()) {
            if (!autonRun) {
                // auton goes here
            }

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }

    class DetectBeaconPipeline extends OpenCvPipeline {
        int latest_x, latest_y;
        boolean viewportPaused;
        String region;

        public DetectBeaconPipeline() {}

        public int[] getLatestCenter () {
            return new int[]{this.latest_x, this.latest_y};
        }
        
        @Override
        public Mat processFrame(Mat input) {
            Mat temp = new Mat();
            Mat mask = new Mat();
            Mat res = new Mat();

            Scalar lowerVal = new Scalar(0, 0, 150);
            Scalar upperVal = new Scalar(100, 100, 255);

            input.copyTo(temp);

            Core.inRange(temp, lowerVal, upperVal, mask);

            Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
            Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

            input.copyTo(res, mask);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
                telemetry.addData("x: ", centerX);
                telemetry.addData("y: ", centerY);
                Imgproc.circle(res, new Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            }

            if (this.latest_x < 280) {
                section = "left";
            } else if (this.latest_x > 360) {
                section = "right";
            } else {
                section = "middle";
            }

            return res;
        }

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
