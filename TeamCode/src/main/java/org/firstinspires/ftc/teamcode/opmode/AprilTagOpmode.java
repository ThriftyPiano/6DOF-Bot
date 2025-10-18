package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Servo turretServo = null;
    private double turretAngle = 0.5;
    private double lastRelativeAngle = 0;

    @Override
    public void runOpMode() {

        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(turretAngle);

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
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // Framerate increase is likely a bluff
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

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

            double[] undistortedPixels = undistortPixel(detection.center.x, detection.center.y);
            double u_undistorted = undistortedPixels[0];
            double v_undistorted = undistortedPixels[1];

            telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

            double fx = pipeline.newCameraMatrix.get(0, 0)[0];
            double fy = pipeline.newCameraMatrix.get(1, 1)[0];
            double cx = pipeline.newCameraMatrix.get(0, 2)[0];
            double cy = pipeline.newCameraMatrix.get(1, 2)[0];

            // Make this consistent with teamwebcamcalibrations.xml
            // Camera angle is from the horizontal, aimed upwards. Hard-coded focal length
            // Hard code focal lengths from calibration: 786.357, 785.863; changed to work a bit better
            double f_x = 786.357 * 2 / 3;
            double f_y = 785.863 * 2 / 3;

            double[] realWorldPosition = calculateTargetPosition3D(
                    u_undistorted, v_undistorted,
                    f_x, f_y,
                    cx, cy,
                    34.2, 0,
                    75);

            double relativeAngle = Math.toDegrees(Math.atan2(realWorldPosition[1], realWorldPosition[0]));

            // Calculate angle using focal length
            double focalAngle = 90 - Math.toDegrees(Math.atan2((detection.center.x - cx), f_x));

            // Add telemetry for focalAngle
            telemetry.addData("Focal Angle", String.format("%6.2f", focalAngle));

            if (detection.id == 20) {
                if (Math.abs(lastRelativeAngle - relativeAngle) > 2) {
                    turretAngle = turretAngle - (relativeAngle - 90) / 300;

                    // Set servo to turn to April tag; estimate 300 degrees of DOF
                    turretServo.setPosition(turretAngle);
                }
                lastRelativeAngle = relativeAngle;
            }

            telemetry.addData("Turret Angle", String.format("%6.2f", turretAngle));

            telemetry.addData("Relative Angle", String.format("%6.2f", relativeAngle));

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
        double x_cam = (u_p - c_x) / f_x;
        double y_cam = -(v_p - c_y) / f_y;
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
        public Mat cameraMatrix;
        public Mat distCoeffs;
        public Mat newCameraMatrix;

        public double[] cameraMatrixArray;

        private org.opencv.core.Size imageSize = new org.opencv.core.Size(1280, 720); // adjust to your camera stream resolution

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            // Do lazy initialization
            if (cameraMatrix == null) {
                // Initialize distortion parameters (you need to calibrate beforehand)
                cameraMatrix = new Mat(3, 3, CvType.CV_64F);
                distCoeffs = new Mat(1, 5, CvType.CV_64F);

                // Example values — replace with your own calibrated ones; these are for 1920x1080 (scaled to 1280x720)
                cameraMatrix.put(0, 0, 1023.878f); // fx
                cameraMatrix.put(0, 1, 0);
                cameraMatrix.put(0, 2, 989.731f * 2 / 3);
                cameraMatrix.put(1, 0, 0);
                cameraMatrix.put(1, 1, 1019.899f); // fy
                cameraMatrix.put(1, 2, 501.663f * 2 / 3);
                cameraMatrix.put(2, 0, 0);
                cameraMatrix.put(2, 1, 0);
                cameraMatrix.put(2, 2, 1);

                // Example distortion coefficients: k1, k2, p1, p2, k3
                distCoeffs.put(0, 0, -0.370767f, 0.106516f, 0.000118f, -0.000542f, -0.012277f);

                //newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(
                //        cameraMatrix, distCoeffs, imageSize, 1, imageSize, null);

                // void newCameraMatrix
                newCameraMatrix = cameraMatrix;
            }

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
    /**
     * Undistorts a single pixel coordinate using the camera's calibration parameters.
     * @param u_distorted The x-coordinate of the distorted pixel.
     * @param v_distorted The y-coordinate of the distorted pixel.
     * @return A double array containing the [x, y] coordinates of the undistorted pixel.
     */
    private double[] undistortPixel(double u_distorted, double v_distorted) {
        // Get camera calibration parameters from the pipeline
        double fx = pipeline.cameraMatrix.get(0, 0)[0];
        double fy = pipeline.cameraMatrix.get(1, 1)[0];
        double cx = pipeline.cameraMatrix.get(0, 2)[0];
        double cy = pipeline.cameraMatrix.get(1, 2)[0];

        // Get distortion coefficients
        double k1 = pipeline.distCoeffs.get(0, 0)[0];
        double k2 = pipeline.distCoeffs.get(0, 1)[0];
        double p1 = pipeline.distCoeffs.get(0, 2)[0];
        double p2 = pipeline.distCoeffs.get(0, 3)[0];
        double k3 = pipeline.distCoeffs.get(0, 4)[0];

        // Normalize distorted pixel coordinates
        double x_distorted = (u_distorted - cx) / fx;
        double y_distorted = (v_distorted - cy) / fy;

        // Iteratively solve for undistorted coordinates using Newton-Raphson method
        double x_undistorted = x_distorted;
        double y_undistorted = y_distorted;

        for (int i = 0; i < 5; i++) {
            double r_sq = x_undistorted * x_undistorted + y_undistorted * y_undistorted;
            double r_fourth = r_sq * r_sq;

            // Radial distortion factor
            double radial = 1.0 + k1 * r_sq + k2 * r_fourth + k3 * r_sq * r_fourth;

            // Tangential distortion factors
            double tangential_x = 2.0 * p1 * x_undistorted * y_undistorted + p2 * (r_sq + 2.0 * x_undistorted * x_undistorted);
            double tangential_y = p1 * (r_sq + 2.0 * y_undistorted * y_undistorted) + 2.0 * p2 * x_undistorted * y_undistorted;

            // Distorted coordinates based on current estimate
            double x_dist_est = x_undistorted * radial + tangential_x;
            double y_dist_est = y_undistorted * radial + tangential_y;

            // Update estimate
            x_undistorted = x_distorted - (x_dist_est - x_undistorted);
            y_undistorted = y_distorted - (y_dist_est - y_undistorted);
        }

        // Convert back to pixel coordinates using the new camera matrix
        double u_undistorted = x_undistorted * pipeline.newCameraMatrix.get(0, 0)[0] + pipeline.newCameraMatrix.get(0, 2)[0];
        double v_undistorted = y_undistorted * pipeline.newCameraMatrix.get(1, 1)[0] + pipeline.newCameraMatrix.get(1, 2)[0];

        return new double[]{u_undistorted, v_undistorted};
    }

}   // end class
