package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

/**
 * VisionArtifactDetector identifies game elements (artifacts) using color segmentation
 * and calculates their relative position to the camera.
 */
public class VisionArtifactDetector {

    public static class Artifact {
        public String type; // e.g. "Green Ball", "Purple Ball"
        public Point pixelPoint;
        public Rect boundingBox;
        public double relX; // Forward distance from camera (inches)
        public double relY; // Left/Right distance from camera (inches)

        public Artifact(String type, Point pixelPoint, Rect boundingBox, double relX, double relY) {
            this.type = type;
            this.pixelPoint = pixelPoint;
            this.boundingBox = boundingBox;
            this.relX = relX;
            this.relY = relY;
        }

        @Override
        public String toString() {
            return String.format("%s: dist=%.1f\", lat=%.1f\" (px: %.0f, %.0f)", 
                                type, relX, relY, pixelPoint.x, pixelPoint.y);
        }
    }

    public static class ColorRange {
        public String label;
        public Scalar lower;
        public Scalar upper;

        public ColorRange(String label, Scalar lower, Scalar upper) {
            this.label = label; this.lower = lower; this.upper = upper;
        }
    }

    private final List<ColorRange> colorRanges = new ArrayList<>();
    private Mat cameraMatrix;
    private VisionLocalizer.VisionPose3D cameraTransform;

    public VisionArtifactDetector(Mat cameraMatrix, VisionLocalizer.VisionPose3D cameraTransform) {
        this.cameraMatrix = cameraMatrix;
        this.cameraTransform = cameraTransform;
        
        // Balanced color ranges for detection and noise reduction
        colorRanges.add(new ColorRange("Green Ball", new Scalar(40, 80, 50), new Scalar(90, 255, 255)));
        colorRanges.add(new ColorRange("Purple Ball", new Scalar(130, 80, 40), new Scalar(170, 255, 255)));
    }

    // Area thresholds as fraction of total frame pixels (resolution-agnostic)
    private static final double MIN_AREA_FRACTION = 0.00033; // ~300px at 1280x720
    private static final double MAX_AREA_FRACTION = 0.022;   // ~20000px at 1280x720

    public List<Artifact> detect(Mat frame) {
        List<Artifact> results = new ArrayList<>();
        if (frame == null || frame.empty()) return results;

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        // Get camera intrinsics
        double fx = cameraMatrix.get(0, 0)[0];
        double fy = cameraMatrix.get(1, 1)[0];
        double cx = cameraMatrix.get(0, 2)[0];
        double cy = cameraMatrix.get(1, 2)[0];

        double totalPixels = frame.cols() * frame.rows();
        double minArea = MIN_AREA_FRACTION * totalPixels;
        double maxArea = MAX_AREA_FRACTION * totalPixels;

        for (ColorRange range : colorRanges) {
            Mat mask = new Mat();
            Core.inRange(hsv, range.lower, range.upper, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > minArea && area < maxArea) {
                    Rect rect = Imgproc.boundingRect(contour);
                    double aspectRatio = (double)rect.width / rect.height;
                    if (aspectRatio > 0.6 && aspectRatio < 1.6) {
                        Point center = new Point(rect.x + rect.width/2.0, rect.y + rect.height/2.0);
                        
                        // --- Projection to Ground ---
                        // 1. Calculate normalized coordinates
                        double nx = (center.x - cx) / fx;
                        double ny = (center.y - cy) / fy;
                        
                        // 2. Project onto ground (assuming camera is at height cameraTransform.z)
                        // In FLU: X=Forward, Y=Left, Z=Up.
                        // For a flat camera: 
                        // Forward distance X = Camera_Height / (Angle_below_horizon)
                        // ny is essentially tan(angle_below_horizon) if camera is flat.
                        // If ny is positive, it's below the center (down).
                        
                        double relX = 0;
                        double relY = 0;
                        
                        if (ny > 0) { // Only project if below horizon
                            // Distance = height / tan(angle)
                            relX = cameraTransform.z / ny;
                            relY = -nx * relX; // Left/Right distance
                        }

                        results.add(new Artifact(range.label, center, rect, relX, relY));
                    }
                }
                contour.release();
            }
            mask.release();
            hierarchy.release();
            kernel.release();
        }
        hsv.release();
        return results;
    }
}
