package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.highgui.HighGui;
import java.util.List;

/**
 * Video-based test for VisionArtifactDetector.
 */
public class VisionArtifactDetectorVideoTest {

    static {
        String arch = System.getProperty("os.arch").toLowerCase();
        String libSuffix = arch.contains("aarch64") || arch.contains("arm64") ? "_arm64" : "_x64";
        String libPath = System.getProperty("user.dir") + "/TeamCode/libs/desktop/libopencv_java490" + libSuffix + ".dylib";
        System.load(libPath);
    }

    public static void main(String[] args) {
        String videoPath = System.getProperty("user.dir") + "/data/vision_artifact_detector/arducam1.mp4";
        VideoCapture cap = new VideoCapture(videoPath);
        
        if (!cap.isOpened()) {
            System.err.println("Error: Could not open video file: " + videoPath);
            return;
        }

        // Initialize with calibrated parameters
        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;
        VisionArtifactDetector detector = new VisionArtifactDetector(cameraMatrix, cameraTransform);
        
        Mat frame = new Mat();
        Mat resized = new Mat();

        System.out.println("Processing video for artifacts. Press 'q' to quit, SPACE to pause.");

        boolean isPaused = false;
        while (true) {
            if (!isPaused) {
                if (!cap.read(frame) || frame.empty()) break;
                // Resize to 1280x720 for consistent detection performance
                Imgproc.resize(frame, resized, new Size(1280, 720));

                List<VisionArtifactDetector.Artifact> artifacts = detector.detect(resized);

                for (VisionArtifactDetector.Artifact a : artifacts) {
                    Scalar color = a.type.contains("Green") ? new Scalar(0, 255, 0) : new Scalar(255, 0, 255);
                    Imgproc.rectangle(resized, a.boundingBox.tl(), a.boundingBox.br(), color, 2);
                    
                    // Display relative position info on screen
                    String label = String.format("%s (%.1f, %.1f)", a.type, a.relX, a.relY);
                    Imgproc.putText(resized, label, new Point(a.boundingBox.x, a.boundingBox.y - 5), 
                                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
                }
            }

            // imshow must be called every loop iteration before waitKey
            if (!resized.empty()) {
                HighGui.imshow("Artifact Detection", resized);
            }

            int key = HighGui.waitKey(30);
            if (key == 'q' || key == 27) break;
            if (key == ' ') isPaused = !isPaused;
        }

        cap.release();
        HighGui.destroyAllWindows();
        System.exit(0);
    }
}
