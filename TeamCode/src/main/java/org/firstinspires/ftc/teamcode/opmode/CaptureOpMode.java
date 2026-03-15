package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Capture Image", group = "Vision")
public class CaptureOpMode extends LinearOpMode {

    private volatile Mat latestFrame = null;
    private volatile boolean captured = false;

    @Override
    public void runOpMode() {
        FrameGrabber grabber = new FrameGrabber();

        com.acmerobotics.dashboard.FtcDashboard dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(grabber)
                .build();

        dashboard.startCameraStream(portal, 0);

        telemetry.addData("Status", "Initialized. Press Start to capture.");
        telemetry.update();

        waitForStart();

        // Wait for a frame to arrive (up to 10 seconds)
        telemetry.addData("Status", "Waiting for camera frame...");
        telemetry.update();

        long deadline = System.currentTimeMillis() + 10000;
        while (opModeIsActive() && latestFrame == null && System.currentTimeMillis() < deadline) {
            sleep(100);
        }

        if (latestFrame != null && !latestFrame.empty()) {
            // VisionPortal delivers RGB frames; Imgcodecs.imwrite expects BGR
            Mat bgrFrame = new Mat();
            Imgproc.cvtColor(latestFrame, bgrFrame, Imgproc.COLOR_RGB2BGR);
            String path = "/sdcard/FIRST/capture_artifacts.jpg";
            boolean ok = Imgcodecs.imwrite(path, bgrFrame);
            captured = true;
            telemetry.addData("Saved", ok ? path : "FAILED to write");
            telemetry.addData("Size", latestFrame.cols() + "x" + latestFrame.rows());
        } else {
            telemetry.addData("Error", "No frame received within 10s");
        }
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }

        portal.close();
    }

    private class FrameGrabber implements VisionProcessor {
        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            if (!captured) {
                latestFrame = input.clone();
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    }
}
