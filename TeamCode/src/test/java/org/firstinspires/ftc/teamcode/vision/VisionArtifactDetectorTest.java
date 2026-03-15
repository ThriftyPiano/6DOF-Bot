package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import java.util.List;

/**
 * PC Test for VisionArtifactDetector using a real captured image.
 */
public class VisionArtifactDetectorTest {

    static {
        String os = System.getProperty("os.name").toLowerCase();
        String arch = System.getProperty("os.arch").toLowerCase();
        String libSuffix = arch.contains("aarch64") || arch.contains("arm64") ? "_arm64" : "_x64";
        String ext = os.contains("mac") ? ".dylib" : ".so";
        String libPath = System.getProperty("user.dir") + "/TeamCode/libs/desktop/libopencv_java490" + libSuffix + ext;

        System.out.println("Loading native library from: " + libPath);
        System.load(libPath);
    }

    public static void main(String[] args) {
        System.out.println("=== VisionArtifactDetector Image Test ===");

        // Load captured image with artifacts
        String imagePath = System.getProperty("user.dir") + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/capture_artifacts.jpg";
        Mat frame = Imgcodecs.imread(imagePath);
        if (frame.empty()) {
            System.err.println("Could not load: " + imagePath);
            return;
        }
        System.out.println("Image size: " + frame.cols() + "x" + frame.rows());

        // The current capture_artifacts.jpg was saved without RGB->BGR conversion,
        // so R and B channels are swapped. Fix it here.
        // Remove this once a new image is captured with the fixed CaptureOpMode.
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2BGR);
        System.out.println("Applied RGB->BGR channel swap (legacy capture fix)");

        // Resize to calibration resolution
        Mat resized = new Mat();
        Imgproc.resize(frame, resized, new Size(1280, 720));

        // Initialize detector with real camera parameters
        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;
        VisionArtifactDetector detector = new VisionArtifactDetector(cameraMatrix, cameraTransform);

        // Run detection
        List<VisionArtifactDetector.Artifact> artifacts = detector.detect(resized);

        System.out.println("Artifacts detected: " + artifacts.size());
        for (VisionArtifactDetector.Artifact a : artifacts) {
            System.out.println("  " + a.toString());
            System.out.printf("    BBox: (%d,%d) %dx%d\n",
                a.boundingBox.x, a.boundingBox.y, a.boundingBox.width, a.boundingBox.height);
        }

        // Ground truth: [forward cm, lateral cm] relative to camera
        // Positive lateral = right of camera
        // Convert cm to inches for comparison (1 inch = 2.54 cm)
        double[][] groundTruthCm = {
            {63, 50},    // 63cm forward, 50cm right
            {63, -57},   // 63cm forward, 57cm left
            {73, 74},    // 73cm forward, 74cm right
            {88, 47},    // 88cm forward, 47cm right
        };

        System.out.println("\nGround Truth (cm -> inches):");
        for (int i = 0; i < groundTruthCm.length; i++) {
            double gtFwd = groundTruthCm[i][0] / 2.54;
            double gtLat = groundTruthCm[i][1] / 2.54;
            System.out.printf("  #%d: fwd=%.1f\" lat=%.1f\" (%.0fcm, %.0fcm)\n",
                i + 1, gtFwd, gtLat, groundTruthCm[i][0], groundTruthCm[i][1]);
        }

        if (artifacts.isEmpty()) {
            System.out.println("\nNo artifacts detected in the image.");
        } else if (artifacts.size() == groundTruthCm.length) {
            // Sort artifacts by pixel X to match left-to-right ground truth ordering
            artifacts.sort((a, b) -> Double.compare(a.pixelPoint.x, b.pixelPoint.x));
            // Re-sort ground truth by lateral position (most left first)
            java.util.Arrays.sort(groundTruthCm, (a, b) -> Double.compare(a[1], b[1]));

            System.out.println("\nComparison (sorted left-to-right):");
            for (int i = 0; i < artifacts.size(); i++) {
                VisionArtifactDetector.Artifact a = artifacts.get(i);
                double gtFwd = groundTruthCm[i][0] / 2.54;
                double gtLat = groundTruthCm[i][1] / 2.54;
                double errFwd = a.relX - gtFwd;
                double errLat = a.relY - (-gtLat); // relY is positive-left, gt lateral is positive-right
                System.out.printf("  %s: fwd=%.1f\" (gt=%.1f\", err=%.1f\"), lat=%.1f\" (gt=%.1f\", err=%.1f\")\n",
                    a.type, a.relX, gtFwd, errFwd, a.relY, -gtLat, errLat);
            }
        } else {
            System.out.printf("\nDetected %d artifacts but have %d ground truth entries.\n",
                artifacts.size(), groundTruthCm.length);
        }

        System.out.println("\nDone.");
    }
}
