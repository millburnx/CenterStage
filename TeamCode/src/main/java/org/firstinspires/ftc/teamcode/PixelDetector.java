package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class PixelDetector{
    OpenCvWebcam webcam;
    ObjectDetectorPipeline pipeline;
    private int latest_x, latest_y;

    public PixelDetector (HardwareMap hardwareMap, Telemetry telemetry) {

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
    }

    public int getX() {
        return latest_x;
    }

    class ObjectDetectorPipeline extends OpenCvPipeline {
        boolean viewportPaused;
        Mat mask, temp, res, kernel, hierarchy;
        ArrayList<MatOfPoint> contours;

        public Mat processFrame(Mat input) {
            temp = new Mat();
            Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

            mask = new Mat();
            res = new Mat();
            hierarchy = new Mat();
            Scalar lowerVal = new Scalar(0, 0, 205);
            Scalar upperVal = new Scalar(255, 50, 255);

            Core.inRange(temp, lowerVal, upperVal, mask);

            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
            Imgproc.erode(mask, mask, kernel);
            Imgproc.dilate(mask, mask, kernel);

            Core.bitwise_and(input, input, res, mask);

            contours = new ArrayList<>();


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
                latest_x = centerX;
                latest_y = centerY;
                Imgproc.circle(res, new Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            }

            mask.release();
            temp.release();
            hierarchy.release();
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
