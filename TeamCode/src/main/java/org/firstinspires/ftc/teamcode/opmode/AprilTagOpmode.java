package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

import java.util.List;

@TeleOp(name = "April Tag Opmode", group = "Concept")

public class AprilTagOpmode extends LinearOpMode {
    private UndistortPipeline pipeline;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        pipeline = new UndistortPipeline();

        builder.addProcessor(pipeline);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
//        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

            double fx = pipeline.newCameraMatrix.get(0, 0)[0];
            double fy = pipeline.newCameraMatrix.get(1, 1)[0];
            double cx = pipeline.newCameraMatrix.get(0, 2)[0];
            double cy = pipeline.newCameraMatrix.get(1, 2)[0];

            // Make this consistent with teamwebcamcalibrations.xml
            // Camera angle is from the horizontal, aimed upwards.
            double[] realWorldPosition = calculateTargetPosition3D(
                    detection.center.x, detection.center.y,
                    fx, fy,
                    cx, cy,
                    31.5, 0.6108,
                    75);

            double relativeAngle = Math.atan2(realWorldPosition[1], realWorldPosition[0]);
            telemetry.addData("Relative Angle", String.format("%6.2f", Math.toDegrees(relativeAngle)));

            telemetry.addLine(String.format("Real World Position %6.2f %6.2f", realWorldPosition[0], realWorldPosition[1]));
        }   // end for() loop

    }   // end method telemetryAprilTag()

    public double[] calculateTargetPosition3D(
            // u_p, v_p is pixel position, c_x, c_y is principal point, f_x, f_y is focal length
            double u_p, double v_p,
            double f_x, double f_y,
            double c_x, double c_y,
            double cameraHeight, double cameraAngleRad,
            double targetHeight
    ) {
        // Convert pixel to normalized camera coordinates
        double x_cam = (u_p - c_x) / f_x * 2;
        double y_cam = -(v_p - c_y) / f_y * 2;
        double z_cam = 1.0;

        // Print all three in one telemetry statement
        telemetry.addData("Unrotated vector", String.format("%6.2f %6.2f %6.2f", x_cam, y_cam, z_cam));

        // Rotate by camera tilt
        double cos_tilt = Math.cos(cameraAngleRad);
        double sin_tilt = Math.sin(cameraAngleRad);
        double x_world_dir = x_cam;
        double y_world_dir = y_cam * cos_tilt + z_cam * sin_tilt;
        double z_world_dir = -y_cam * sin_tilt + z_cam * cos_tilt;

        // Print rotated vector
        telemetry.addData("Rotated vector", String.format("%6.2f %6.2f %6.2f", x_world_dir, y_world_dir, z_world_dir));

        // Find ray intersection with ground plane
        if (Math.abs(y_world_dir) < 1e-9) {
            return new double[]{0.0, 0.0};
        }
        double t = (targetHeight - cameraHeight) / y_world_dir;

        telemetry.addData("t", t);

        if (t <= 0) {
            return new double[]{0.0, 0.0};
        }

        // Calculate ground position
        double x_world = t * x_world_dir;
        double z_world = t * z_world_dir;

        return new double[]{x_world, z_world};
    }

    public class UndistortPipeline implements VisionProcessor {
        private Mat cameraMatrix;
        private Mat distCoeffs;
        public Mat newCameraMatrix;

        public double[] cameraMatrixArray;

        private org.opencv.core.Size imageSize = new org.opencv.core.Size(1920, 1080); // adjust to your camera stream resolution

        private boolean initialized = false;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Initialize distortion parameters (you need to calibrate beforehand)
            cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            distCoeffs = new Mat(1, 5, CvType.CV_64F);

            // Example values — replace with your own calibrated ones
            cameraMatrix.put(0, 0, 1023.878f); // fx
            cameraMatrix.put(0, 1, 0);
            cameraMatrix.put(0, 2, 989.731f);
            cameraMatrix.put(1, 0, 0);
            cameraMatrix.put(1, 1, 1019.899f); // fy
            cameraMatrix.put(1, 2, 501.663f);
            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2, 1, 0);
            cameraMatrix.put(2, 2, 1);

            // Example distortion coefficients: k1, k2, p1, p2, k3
            distCoeffs.put(0, 0, -0.370767f, 0.106516f, 0.000118f, -0.000542f, -0.012277f);

            newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(
                    cameraMatrix, distCoeffs, imageSize, 1, imageSize, null);

            // void newCameraMatrix
            newCameraMatrix = cameraMatrix;

            initialized = true;
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            if (!initialized) return null;

            Mat undistorted = new Mat();
            Calib3d.undistort(input, undistorted, cameraMatrix, distCoeffs, newCameraMatrix);

            // (Optional) Do further image processing here
            // e.g. detect apriltags, contours, colors, etc.

            // Replace the input with undistorted image for display in the RC preview
            undistorted.copyTo(input);
            undistorted.release();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // You can draw overlays here if you want (optional)
        }
    }

}   // end class
