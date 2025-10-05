package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "AprilTag Detection Example")
public class AprilTagDetectionOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagPipeline;
    AprilTagDetectionPipeline.Point2d pos;

    // camera_matrix = np.array([
    //     [691.93178295137910, 0.0, 623.4774035905616],
    //     [0.0, 695.70002287015314, 346.47156326286608],
    //     [0.0, 0.0, 1.0]
    // ], dtype=np.float32)
    double fx = 691.93178295137910; // pixels
    double fy = 695.70002287015314; // pixels
    double cx = 623.4774035905616; // pixels
    double cy = 346.47156326286608; // pixels

    // All other units in centimeters
    double cameraHeight = 20.0; // cm (example, set to your camera's height above ground)
    double cameraAngleRad = Math.toRadians(30.0); // example tilt, adjust as needed
    double tagHeight = 75.0; // cm (tag center height above ground)

    // AprilTag size: 8.125 inches = 20.6375 cm
    double tagSize = 20.6375; // cm

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    aprilTagPipeline = new AprilTagDetectionPipeline(tagSize / 100.0, fx, fy, cx, cy);
    webcam.setPipeline(aprilTagPipeline);
    webcam.openCameraDeviceAsync(() -> webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT));

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetectionPipeline.Detection> detections = aprilTagPipeline.getDetections();
            if (detections != null) {
                for (AprilTagDetectionPipeline.Detection detection : detections) {
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Center (pixels)", "%f, %f", detection.centre.x, detection.centre.y);
                    // Real-world position (meters)
                    telemetry.addData("Translation X", detection.pose.x);
                    telemetry.addData("Translation Y", detection.pose.y);
                    telemetry.addData("Translation Z", detection.pose.z);
                    // Pixel-to-real-world conversion using pipeline
                    pos = aprilTagPipeline.calculateTagPosition3D(detection.centre.x, detection.centre.y);
                    telemetry.addData("Pipeline Real X (cm, ground)", pos.x);
                    telemetry.addData("Pipeline Real Y (cm, ground)", pos.y);
                    telemetry.addData("Pipeline Real Z (cm, tag height)", tagHeight);
                }
            }
            telemetry.update();
            sleep(20);
        }
    }
}
