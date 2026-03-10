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
import org.firstinspires.ftc.teamcode.vision.VisionArtifactDetector;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

@TeleOp(name = "Vision OpMode (Localizer + Artifacts)", group = "Vision")
public class VisionOpMode extends LinearOpMode {

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
        // dashboard.startCameraStream(visionPortal, 0); // temporarily disabled
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

            // Per-tag camera poses
            Map<Integer, VisionLocalizer.VisionPose3D> perTagPoses = localizer.estimateCameraPosePerTag(localizerDetections);
            VisionLocalizer.VisionPose3D bluePose = perTagPoses.get(20);
            VisionLocalizer.VisionPose3D redPose = perTagPoses.get(24);
            VisionLocalizer.VisionPose3D cameraPose = localizer.estimateCameraPose(localizerDetections);

            // 2. Dashboard TelemetryPacket with Field Overlay
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Tags Detected", currentDetections.size());
            com.acmerobotics.dashboard.canvas.Canvas fieldOverlay = packet.fieldOverlay();

            if (bluePose != null) {
                drawPose(fieldOverlay, bluePose, "#0000FF");
                packet.put("Blue (Tag 20)", bluePose.toString());
                packet.put("Blue X", bluePose.x);
                packet.put("Blue Y", bluePose.y);
                packet.put("Blue Z", bluePose.z);
            }
            if (redPose != null) {
                drawPose(fieldOverlay, redPose, "#FF0000");
                packet.put("Red (Tag 24)", redPose.toString());
                packet.put("Red X", redPose.x);
                packet.put("Red Y", redPose.y);
                packet.put("Red Z", redPose.z);
            }
            if (cameraPose != null) {
                drawPose(fieldOverlay, cameraPose, "#00FF00");
                packet.put("Camera Pose", cameraPose.toString());
                packet.put("Cam X", cameraPose.x);
                packet.put("Cam Y", cameraPose.y);
                packet.put("Cam Z", cameraPose.z);
            } else {
                packet.put("Camera Pose", "Searching for tags...");
            }

            List<VisionArtifactDetector.Artifact> artifacts = artifactProcessor.getLatestArtifacts();
            packet.put("Artifacts Detected", artifacts.size());

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
        private VisionArtifactDetector detector;
        private List<VisionArtifactDetector.Artifact> latestArtifacts = new ArrayList<>();
        private Mat resized = new Mat();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            detector = new VisionArtifactDetector(VisionLocalizer.getArducamMatrix(), VisionLocalizer.ARDUCAM_TRANSFORM);
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            // Resize to 1280x720 to match calibration
            localizer.prepareFrame(input, resized);

            // Run Detection
            latestArtifacts = detector.detect(resized);

            // Draw detections on the original input mat (for the RC preview)
            // Note: input is 1920x1080, detections are in 1280x720 space.
            double scale = 1920.0 / 1280.0;
            for (VisionArtifactDetector.Artifact a : latestArtifacts) {
                Scalar color = a.type.contains("Green") ? new Scalar(0, 255, 0) : new Scalar(255, 0, 255);
                Point p1 = new Point(a.boundingBox.x * scale, a.boundingBox.y * scale);
                Point p2 = new Point((a.boundingBox.x + a.boundingBox.width) * scale, (a.boundingBox.y + a.boundingBox.height) * scale);
                Imgproc.rectangle(input, p1, p2, color, 4);
            }

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

        public List<VisionArtifactDetector.Artifact> getLatestArtifacts() {
            return latestArtifacts;
        }
    }
}
