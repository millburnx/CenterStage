package main.java.org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.*;

import java.util.ArrayList;


public class ObjectDetector {
    private HardwareMap hm;
    private Telemetry telemetry;
    public OpenCvWebcam camera;
    public ObjectDetectionPipeline odp;
    public ObjectDetector(HardwareMap hm, Telemetry tm)
    {
        this.telemetry = tm;
        this.hm = hm;
        int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        odp = new ObjectDetectionPipeline();
        camera.setPipeline(odp);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                camera.showFpsMeterOnViewport(true);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });
    }
    public int[] search()
    {
        return odp.getLatestCenter();
    }
    private static class ObjectDetectionPipeline extends OpenCvPipeline
    {
        int latest_x;
        int latest_y;
        public ObjectDetectionPipeline()
        {

        }
        public int[] getLatestCenter()
        {
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
                Imgproc.circle(res, new Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            }
            return res;
        }

    }
}