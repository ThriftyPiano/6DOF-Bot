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
    
    private class TagPosition {
        public final double x;
        public final double y;
        public final double relativeAngle;
        public final double distance;
        public final double tagHeight;

        public TagPosition(double x, double y, double relativeAngle, double distance, double tagHeight) {
            this.x = x;
            this.y = y;
            this.relativeAngle = relativeAngle;
            this.distance = distance;
            this.tagHeight = tagHeight;
        }
    }
    private UndistortPipeline pipeline;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Servo turretServo = null;
    private double turretAngle = 0.576;
    // 180 / ([90 deg pos] - [-90 deg pos]); in degrees / servo value
    private double turretSpeed = 180 / (0.901 - 0.239);
    // private double turretSpeed = 250;
    private double lastRelativeAngle = 0;

    // Record number of consecutive turns as metric to prevent oscillations
    private int consecutiveTurns = 0;

    /**
     * Calculates 3D position of an AprilTag based on its detection
     * @param detection The AprilTag detection
     * @return TagPosition object containing position and angle information
     */
    private TagPosition calculateTagPosition3D(AprilTagDetection detection) {
        // Calculate height using corners
        org.opencv.core.Point[] corners = detection.corners;
        double tagHeight = Math.abs((corners[0].y + corners[1].y) - (corners[2].y + corners[3].y)) / 2;
        
        // Calculate angle using focal length
        double relativeAngle = 90 - Math.toDegrees(Math.atan2((detection.center.x - c_x_afterresize), f_x_afterresize));
        
        // Calculate position based on size
        double groundDistance = tagSizeCm / tagHeight * f_y_afterresize;
        double x = groundDistance * Math.cos(Math.toRadians(relativeAngle));
        double y = groundDistance * Math.sin(Math.toRadians(relativeAngle));
        
        return new TagPosition(x, y, relativeAngle, groundDistance, tagHeight);
    }

    // Make this consistent with teamwebcamcalibrations.xml
    // Camera angle is from the horizontal, aimed upwards. Hard-coded focal length
    // Hard code focal lengths from calibration: 786.357, 785.863; scale down to 1280x720 from 1920x1080
    private double f_x_afterresize = 786.357 * 2 / 3;
    private double f_y_afterresize = 785.863 * 2 / 3;
    private double c_x_afterresize = 989.731f * 2 / 3;
    private double c_y_afterresize = 501.663f * 2 / 3;

    // April tag height
    private double tagSizeCm = 16.51; // 6.5 inches

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
        builder.setCameraResolution(new Size(1920, 1080));

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
        //FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

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
            
            TagPosition tagPosition = calculateTagPosition3D(detection);
            
            telemetry.addLine(String.format("Tag Height: %.1f pixels", tagPosition.tagHeight));
            telemetry.addData("Ground Distance", String.format("%6.2f cm", tagPosition.distance));

            // Add telemetry for position and angle
            telemetry.addData("Tag Angle", String.format("%6.2f", tagPosition.relativeAngle));

            // Aim at blue goal tag
            if (detection.id == 20) {
                if (Math.abs(lastRelativeAngle - tagPosition.relativeAngle) > 2) {
                    consecutiveTurns += 1;
                    double turnAmount = -(tagPosition.relativeAngle - 90) / turretSpeed;
                    // Damping to prevent oscillations after more than 3 consecutive turns
                    if (consecutiveTurns > 3) {
                        turnAmount /= 2;
                    }
                    turretAngle = turretAngle + turnAmount;

                    // Set servo to turn to April tag
                    turretServo.setPosition(turretAngle);
                }
                else {
                    consecutiveTurns = 0;
                }
                lastRelativeAngle = tagPosition.relativeAngle;
            }

            telemetry.addData("Relative Angle", String.format("%6.2f", tagPosition.relativeAngle));
            // Add telemetry for calculated positions
            telemetry.addData("Calculated X Pos", String.format("%6.2f", tagPosition.x));
            telemetry.addData("Calculated Y Pos", String.format("%6.2f", tagPosition.y));

        }   // end for() loop

    }   // end method telemetryAprilTag()

    public class UndistortPipeline implements VisionProcessor {
        public Mat cameraMatrix;
        public Mat distCoeffs;
        public Mat newCameraMatrix;

        public double[] cameraMatrixArray;

        private org.opencv.core.Size imageSize = new org.opencv.core.Size(1920, 1080); // adjust to your camera stream resolution

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
                cameraMatrix.put(0, 2, 989.731f);
                cameraMatrix.put(1, 0, 0);
                cameraMatrix.put(1, 1, 1019.899f); // fy
                cameraMatrix.put(1, 2, 501.663f);
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

            // Scale the image down to 1280x720 after undistortion
            Mat resized = new Mat();
            org.opencv.core.Size newSize = new org.opencv.core.Size(1280, 720);
            Imgproc.resize(undistorted, resized, newSize);

            // Draw a blue dot at the principal point using pre-calculated after-resize coordinates
            // Imgproc.circle(resized, new Point(c_x_afterresize, c_y_afterresize), 5, new Scalar(0, 0, 255), -1);

            // (Optional) Do further image processing here
            // e.g. detect apriltags, contours, colors, etc.

            // Replace the input with resized image for display in the RC preview
            resized.copyTo(input);
            resized.release();
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
