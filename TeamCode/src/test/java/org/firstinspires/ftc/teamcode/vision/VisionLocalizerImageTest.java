package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class VisionLocalizerImageTest {

    static {
        String os = System.getProperty("os.name").toLowerCase();
        String arch = System.getProperty("os.arch").toLowerCase();
        String libSuffix = arch.contains("aarch64") || arch.contains("arm64") ? "_arm64" : "_x64";
        String ext = os.contains("mac") ? ".dylib" : ".so";
        
        String libPath = System.getProperty("user.dir") + "/TeamCode/libs/desktop/libopencv_java490" + libSuffix + ext;
        System.out.println("Loading " + os + " (" + arch + ") native library from: " + libPath);
        
        try {
            System.load(libPath);
            System.out.println("Successfully loaded OpenCV native library.");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Failed to load OpenCV native library at " + libPath);
            throw e;
        }
    }

    static class TestCase {
        String name;
        String path;
        double gtX, gtY;
        double simulatedYaw;

        TestCase(String name, String path, double gtX, double gtY, double simulatedYaw) {
            this.name = name;
            this.path = path;
            this.gtX = gtX;
            this.gtY = gtY;
            this.simulatedYaw = simulatedYaw;
        }
    }

    public static void main(String[] args) {
        String testDataDir = System.getProperty("user.dir") + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/";

        List<TestCase> tests = new ArrayList<>();
        // Camera at field origin (0,0), 44cm height, facing the tags
        tests.add(new TestCase("Capture Field", testDataDir + "capture_field.jpg", 0.0, 0.0, 0.0));

        // Setup Localizer
        Map<Integer, VisionLocalizer.VisionPose3D> fieldMap = VisionLocalizer.DECODE_FIELD_MAP;
        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        MatOfDouble distCoeffs = VisionLocalizer.getArducamDistCoeffs();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;
        VisionLocalizer localizer = new VisionLocalizer(fieldMap, cameraMatrix, distCoeffs, cameraTransform);

        // Setup Detector
        Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11);
        ArucoDetector detector = new ArucoDetector(dictionary);

        System.out.println("=== VisionLocalizer Camera Pose Test ===");
        System.out.println("Using Tag Size: " + VisionLocalizer.TAG_SIZE_INCHES + " inches");
        System.out.println("Field Map:");
        for (Map.Entry<Integer, VisionLocalizer.VisionPose3D> entry : fieldMap.entrySet()) {
            System.out.println("  Tag " + entry.getKey() + ": " + entry.getValue());
        }

        for (TestCase test : tests) {
            System.out.println("\n--- Testing: " + test.name + " ---");
            Mat frame = Imgcodecs.imread(test.path);
            if (frame.empty()) {
                System.err.println("Could not load: " + test.path);
                continue;
            }
            System.out.println("Image size: " + frame.cols() + "x" + frame.rows());

            Mat resized = new Mat();
            localizer.prepareFrame(frame, resized);
            System.out.println("Resized to: " + resized.cols() + "x" + resized.rows());

            List<Mat> corners = new ArrayList<>();
            Mat ids = new Mat();
            detector.detectMarkers(resized, corners, ids);

            List<VisionLocalizer.Detection> detections = new ArrayList<>();
            if (!ids.empty()) {
                for (int i = 0; i < ids.rows(); i++) {
                    int id = (int) ids.get(i, 0)[0];
                    Mat cornerMat = corners.get(i);
                    // ArUco returns corners as BR, BL, TL, TR
                    // Reorder to TL, TR, BR, BL for VisionLocalizer
                    Point[] aruco = new Point[4];
                    for (int j = 0; j < 4; j++) {
                        aruco[j] = new Point(cornerMat.get(0, j)[0], cornerMat.get(0, j)[1]);
                    }
                    Point[] pts = new Point[] { aruco[2], aruco[3], aruco[0], aruco[1] };
                    detections.add(new VisionLocalizer.Detection(id, pts));
                    System.out.printf("  Tag ID %d corners: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n",
                        id, pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x, pts[2].y, pts[3].x, pts[3].y);
                }
            } else {
                System.out.println("  No tags detected!");
            }

            // Per-tag poses
            Map<Integer, VisionLocalizer.VisionPose3D> perTagPoses = localizer.estimateCameraPosePerTag(detections);
            for (Map.Entry<Integer, VisionLocalizer.VisionPose3D> entry : perTagPoses.entrySet()) {
                VisionLocalizer.VisionPose3D p = entry.getValue();
                System.out.printf("  Tag %d -> Camera Pose: X=%.2f, Y=%.2f, Z=%.2f\n", entry.getKey(), p.x, p.y, p.z);
            }

            // Averaged pose
            VisionLocalizer.VisionPose3D cameraPose = localizer.estimateCameraPose(detections);

            if (cameraPose != null) {
                System.out.println("Averaged Camera Pose: " + cameraPose.toString());
                System.out.printf("Ground Truth: X=%.2f, Y=%.2f\n", test.gtX, test.gtY);

                double errX = cameraPose.x - test.gtX;
                double errY = cameraPose.y - test.gtY;
                System.out.printf("Error: dX=%.2f, dY=%.2f, Dist=%.2f\n", errX, errY, Math.sqrt(errX*errX + errY*errY));
            } else {
                System.out.println("Camera pose estimation FAILED.");
            }
        }
        System.out.println("\nDone.");
        System.exit(0);
    }
}
