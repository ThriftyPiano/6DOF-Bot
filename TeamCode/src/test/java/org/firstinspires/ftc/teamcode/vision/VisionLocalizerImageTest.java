package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
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
        double headingRadians; // camera heading in field coordinates

        TestCase(String name, String path, double gtX, double gtY, double headingRadians) {
            this.name = name;
            this.path = path;
            this.gtX = gtX;
            this.gtY = gtY;
            this.headingRadians = headingRadians;
        }
    }

    /**
     * Extract camera heading from detected tags using solvePnP on field-frame points.
     * This simulates what the IMU would provide on the real robot.
     */
    static double extractHeading(List<VisionLocalizer.Detection> detections, Mat cameraMatrix, MatOfDouble distCoeffs) {
        List<Point3> fieldPts = new ArrayList<>();
        List<Point> imgPts = new ArrayList<>();
        for (VisionLocalizer.Detection det : detections) {
            VisionLocalizer.VisionPose3D tagPose = VisionLocalizer.DECODE_FIELD_MAP.get(det.id);
            if (tagPose == null) continue;
            double tagYaw = tagPose.yaw;
            double cosT = Math.cos(tagYaw), sinT = Math.sin(tagYaw);
            double half = VisionLocalizer.TAG_SIZE_INCHES / 2.0;
            double[][] tl = {{-half,-half,0},{half,-half,0},{half,half,0},{-half,half,0}};
            for (int i = 0; i < 4; i++) {
                double lx = tl[i][0], ly = tl[i][1], lz = tl[i][2];
                fieldPts.add(new Point3(
                    tagPose.x + (-sinT)*lx + (-cosT)*lz,
                    tagPose.y + cosT*lx + (-sinT)*lz,
                    tagPose.z + (-1)*ly));
                imgPts.add(new Point(det.corners[i].x, det.corners[i].y));
            }
        }
        MatOfPoint3f objM = new MatOfPoint3f(fieldPts.toArray(new Point3[0]));
        MatOfPoint2f imgM = new MatOfPoint2f(imgPts.toArray(new Point[0]));
        Mat rvec = new Mat(), tvec = new Mat();
        Calib3d.solvePnP(objM, imgM, cameraMatrix, distCoeffs, rvec, tvec);
        Mat R = new Mat();
        Calib3d.Rodrigues(rvec, R);
        // Z_cam direction = row 2 of R = (cos h, sin h, ~0)
        return Math.atan2(R.get(2, 1)[0], R.get(2, 0)[0]);
    }

    public static void main(String[] args) {
        String testDataDir = System.getProperty("user.dir") + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/";

        List<TestCase> tests = new ArrayList<>();
        // Camera at field origin (0,0), 44cm height, facing -X toward the wall with tags
        // Heading ~176.4° determined from solvePnP (camera not perfectly aligned to -X axis)
        tests.add(new TestCase("Capture Field", testDataDir + "capture_field.jpg", 0.0, 0.0, Math.PI));

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

            // Per-tag poses (unconstrained)
            Map<Integer, VisionLocalizer.VisionPose3D> perTagPoses = localizer.estimateCameraPosePerTag(detections);
            for (Map.Entry<Integer, VisionLocalizer.VisionPose3D> entry : perTagPoses.entrySet()) {
                VisionLocalizer.VisionPose3D p = entry.getValue();
                System.out.printf("  Tag %d -> Camera Pose: X=%.2f, Y=%.2f, Z=%.2f\n", entry.getKey(), p.x, p.y, p.z);
            }

            // Averaged pose (unconstrained)
            VisionLocalizer.VisionPose3D cameraPose = localizer.estimateCameraPose(detections);
            if (cameraPose != null) {
                System.out.println("Unconstrained Pose: " + cameraPose.toString());
                double errX = cameraPose.x - test.gtX;
                double errY = cameraPose.y - test.gtY;
                System.out.printf("Unconstrained Error: dX=%.2f, dY=%.2f, Dist=%.2f\n", errX, errY, Math.sqrt(errX*errX + errY*errY));
            }

            // Extract true heading from vision (simulates accurate IMU)
            double trueHeading = extractHeading(detections, cameraMatrix, distCoeffs);
            System.out.printf("Extracted heading: %.2f° (nominal: %.2f°)\n",
                Math.toDegrees(trueHeading), Math.toDegrees(test.headingRadians));

            // MegaTag2: constrained pose with nominal heading
            VisionLocalizer.VisionPose3D mt2Nominal = localizer.estimateCameraPoseConstrained(detections, test.headingRadians);
            if (mt2Nominal != null) {
                System.out.println("MT2 Pose (nominal heading): " + mt2Nominal.toString());
                double errX = mt2Nominal.x - test.gtX;
                double errY = mt2Nominal.y - test.gtY;
                System.out.printf("MT2 Nominal Error:   dX=%.2f, dY=%.2f, Dist=%.2f\n", errX, errY, Math.sqrt(errX*errX + errY*errY));
            }

            // MegaTag2: constrained pose with extracted heading (best case)
            VisionLocalizer.VisionPose3D mt2Best = localizer.estimateCameraPoseConstrained(detections, trueHeading);
            if (mt2Best != null) {
                System.out.println("MT2 Pose (true heading):    " + mt2Best.toString());
                double errX = mt2Best.x - test.gtX;
                double errY = mt2Best.y - test.gtY;
                System.out.printf("MT2 True Error:      dX=%.2f, dY=%.2f, Dist=%.2f\n", errX, errY, Math.sqrt(errX*errX + errY*errY));
            }
        }
        System.out.println("\nDone.");
        System.exit(0);
    }
}
