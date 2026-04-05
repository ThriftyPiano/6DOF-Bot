package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.teamcode.vision.VisionLocalizer;
import org.firstinspires.ftc.teamcode.vision.VisionArtifactDetector2;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

@TeleOp(name = "Vision OpMode (Localizer + Artifacts)", group = "Vision")
public class VisionOpMode extends LinearOpMode {

    // Heading offset: field heading = imuYaw + HEADING_OFFSET
    // Set this to the robot's initial heading in field coordinates (radians, CCW from +X).
    // Example: if robot starts facing -X (toward the tag wall), set to Math.PI.
    private static final double HEADING_OFFSET = Math.PI;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ArtifactProcessor artifactProcessor;
    private VisionLocalizer localizer;
    private IMU imu;

    @Override
    public void runOpMode() {
        // 1. Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // 2. Initialize Localizer Logic
        Map<Integer, VisionLocalizer.VisionPose3D> fieldMap = VisionLocalizer.DECODE_FIELD_MAP;
        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        MatOfDouble distCoeffs = VisionLocalizer.getArducamDistCoeffs();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;
        localizer = new VisionLocalizer(fieldMap, cameraMatrix, distCoeffs, cameraTransform);

        // 3. Initialize Vision Processors
        aprilTag = new AprilTagProcessor.Builder().build();
        artifactProcessor = new ArtifactProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(artifactProcessor)
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Process AprilTags for Camera Localization
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            List<VisionLocalizer.Detection> localizerDetections = new ArrayList<>();
            for (AprilTagDetection det : currentDetections) {
                // FTC SDK returns corners as BL, BR, TR, TL at 1920x1080
                // Reorder to TL, TR, BR, BL and scale to 1280x720 (calibration resolution)
                double scale = 1280.0 / 1920.0;
                Point[] corners = new Point[] {
                    new Point(det.corners[3].x * scale, det.corners[3].y * scale), // TL
                    new Point(det.corners[2].x * scale, det.corners[2].y * scale), // TR
                    new Point(det.corners[1].x * scale, det.corners[1].y * scale), // BR
                    new Point(det.corners[0].x * scale, det.corners[0].y * scale)  // BL
                };
                localizerDetections.add(new VisionLocalizer.Detection(det.id, corners));
            }

            // Per-tag camera poses (unconstrained, for comparison)
            Map<Integer, VisionLocalizer.VisionPose3D> perTagPoses = localizer.estimateCameraPosePerTag(localizerDetections);
            VisionLocalizer.VisionPose3D bluePose = perTagPoses.get(20);
            VisionLocalizer.VisionPose3D redPose = perTagPoses.get(24);

            // MegaTag2: constrained pose using IMU yaw converted to field heading
            double imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double fieldHeading = imuYaw + HEADING_OFFSET;
            VisionLocalizer.VisionPose3D cameraPose = localizer.estimateCameraPoseConstrained(localizerDetections, fieldHeading);

            // 2. Dashboard TelemetryPacket with Field Overlay
            TelemetryPacket packet = new TelemetryPacket(false);

            // Draw DECODE field background rotated 180°
            com.acmerobotics.dashboard.canvas.Canvas field = packet.field();
            field.setAlpha(0.4);
            field.drawImage("/dash/decode.webp", 72, 72, 144, 144, Math.PI, 72, 72, true);
            field.setAlpha(1.0);
            field.drawGrid(0, 0, 144, 144, 7, 7);

            packet.put("Tags Detected", currentDetections.size());
            com.acmerobotics.dashboard.canvas.Canvas fieldOverlay = packet.fieldOverlay();

            // Rotate all annotations 180° around field center
            fieldOverlay.setRotation(Math.PI);

            if (bluePose != null) {
                drawPose(fieldOverlay, bluePose, "#0000FF");
                packet.put("Blue (Tag 20)", bluePose.toString());
            }
            if (redPose != null) {
                drawPose(fieldOverlay, redPose, "#FF0000");
                packet.put("Red (Tag 24)", redPose.toString());
            }
            packet.put("IMU Yaw", Math.toDegrees(imuYaw));
            packet.put("Field Heading", Math.toDegrees(fieldHeading));
            if (cameraPose != null) {
                drawPose(fieldOverlay, cameraPose, "#00FF00");
                packet.put("Camera Pose (MT2)", cameraPose.toString());
            } else {
                packet.put("Camera Pose", "Searching for tags...");
            }

            List<VisionArtifactDetector2.Artifact> artifacts = artifactProcessor.getLatestArtifacts();
            packet.put("Artifacts Detected", artifacts.size());

            // Draw artifacts on field overlay
            if (cameraPose != null) {
                double cosH = Math.cos(fieldHeading);
                double sinH = Math.sin(fieldHeading);
                for (int i = 0; i < artifacts.size(); i++) {
                    VisionArtifactDetector2.Artifact a = artifacts.get(i);
                    if (a.relX <= 0) continue;
                    // Convert camera-relative (forward/lateral) to field coordinates
                    // relX = forward (along camera heading), relY = left (positive)
                    double fieldX = cameraPose.x + a.relX * cosH - a.relY * sinH;
                    double fieldY = cameraPose.y + a.relX * sinH + a.relY * cosH;

                    String color = a.type.contains("Green") ? "#00CC00" : "#CC00CC";
                    fieldOverlay.setStroke(color);
                    fieldOverlay.setFill(color);
                    fieldOverlay.strokeCircle(fieldX, fieldY, 2.5);
                    fieldOverlay.fillCircle(fieldX, fieldY, 2.5);
                    packet.put("Artifact " + i, String.format("%s (%.1f, %.1f)", a.type, fieldX, fieldY));
                }
            }

            dashboard.sendTelemetryPacket(packet);
            sleep(200);
        }

        visionPortal.close();
    }

    private void drawPose(com.acmerobotics.dashboard.canvas.Canvas canvas, VisionLocalizer.VisionPose3D pose, String color) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(pose.x, pose.y, 4.0);
    }

    /**
     * Custom VisionProcessor to run our Artifact Detector inside the VisionPortal pipeline.
     */
    public class ArtifactProcessor implements VisionProcessor {
        private VisionArtifactDetector2 detector;
        private List<VisionArtifactDetector2.Artifact> latestArtifacts = new ArrayList<>();
        private Mat resized = new Mat();
        private Mat cameraMatrix;
        private MatOfDouble distCoeffs;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            detector = new VisionArtifactDetector2(VisionLocalizer.getArducamMatrix(), VisionLocalizer.ARDUCAM_TRANSFORM);
            cameraMatrix = VisionLocalizer.getArducamMatrix();
            distCoeffs = VisionLocalizer.getArducamDistCoeffs();
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            // Resize to 1280x720 to match calibration
            localizer.prepareFrame(input, resized);

            // Run detection on distorted frame (cosine similarity is robust to distortion)
            latestArtifacts = detector.detect(resized);

            // Undistort detected center points for accurate position estimation
            if (!latestArtifacts.isEmpty()) {
                Point[] distorted = new Point[latestArtifacts.size()];
                for (int i = 0; i < latestArtifacts.size(); i++) {
                    distorted[i] = latestArtifacts.get(i).pixelPoint;
                }
                MatOfPoint2f distortedMat = new MatOfPoint2f(distorted);
                MatOfPoint2f undistortedMat = new MatOfPoint2f();
                Calib3d.undistortPoints(distortedMat, undistortedMat, cameraMatrix, distCoeffs, new Mat(), cameraMatrix);
                Point[] undistortedPts = undistortedMat.toArray();
                for (int i = 0; i < latestArtifacts.size(); i++) {
                    latestArtifacts.get(i).pixelPoint = undistortedPts[i];
                }
                distortedMat.release();
                undistortedMat.release();
            }

            // Draw detections on the resized frame
            for (VisionArtifactDetector2.Artifact a : latestArtifacts) {
                Scalar color = a.type.contains("Green") ? new Scalar(0, 255, 0) : new Scalar(255, 0, 255);
                Point p1 = new Point(a.boundingBox.x, a.boundingBox.y);
                Point p2 = new Point(a.boundingBox.x + a.boundingBox.width, a.boundingBox.y + a.boundingBox.height);
                Imgproc.rectangle(resized, p1, p2, color, 3);

                Point center = new Point(a.pixelPoint.x, a.pixelPoint.y);
                Imgproc.circle(resized, center, 6, color, -1);

                String label = String.format("%s %.0f\" fwd %.0f\" lat", a.type, a.relX, a.relY);
                Point textPos = new Point(p1.x, p1.y - 8);
                Imgproc.putText(resized, label, textPos, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }

            // Copy frame back to input for dashboard camera stream
            Imgproc.resize(resized, input, input.size());

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

        public List<VisionArtifactDetector2.Artifact> getLatestArtifacts() {
            return latestArtifacts;
        }
    }
}
