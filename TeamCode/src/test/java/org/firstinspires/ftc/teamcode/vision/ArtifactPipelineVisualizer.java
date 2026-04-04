package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Visualizes each step of the artifact detection pipeline for debugging.
 * Outputs per-image: raw, HSV masks (green/purple), filtered contours, and final detections.
 */
public class ArtifactPipelineVisualizer {

    static {
        String os = System.getProperty("os.name").toLowerCase();
        String arch = System.getProperty("os.arch").toLowerCase();
        String libSuffix = arch.contains("aarch64") || arch.contains("arm64") ? "_arm64" : "_x64";
        String ext = os.contains("mac") ? ".dylib" : ".so";
        String libPath = System.getProperty("user.dir") + "/TeamCode/libs/desktop/libopencv_java490" + libSuffix + ext;
        System.out.println("Loading native library from: " + libPath);
        System.load(libPath);
    }

    // Same color ranges as VisionArtifactDetector
    private static final Scalar GREEN_LOWER = new Scalar(40, 80, 50);
    private static final Scalar GREEN_UPPER = new Scalar(90, 255, 255);
    private static final Scalar PURPLE_LOWER = new Scalar(130, 80, 40);
    private static final Scalar PURPLE_UPPER = new Scalar(170, 255, 255);

    // Same resolution-agnostic fractions as VisionArtifactDetector
    private static final double MIN_AREA_FRACTION = 0.00033;
    private static final double MAX_AREA_FRACTION = 0.022;
    private static final double MIN_ASPECT = 0.6;
    private static final double MAX_ASPECT = 1.6;

    public static void main(String[] args) {
        String baseDir = System.getProperty("user.dir");
        String imageDir = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision";
        String outputDir = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/pipeline_output";

        new File(outputDir).mkdirs();

        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        MatOfDouble distCoeffs = VisionLocalizer.getArducamDistCoeffs();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;

        double fx = cameraMatrix.get(0, 0)[0];
        double fy = cameraMatrix.get(1, 1)[0];
        double cx = cameraMatrix.get(0, 2)[0];
        double cy = cameraMatrix.get(1, 2)[0];

        for (int imgIdx = 1; imgIdx <= 5; imgIdx++) {
            String imagePath = imageDir + "/artifact_img" + imgIdx + ".jpg";
            Mat raw = Imgcodecs.imread(imagePath);
            if (raw.empty()) {
                System.err.println("Could not load: " + imagePath);
                continue;
            }
            System.out.println("\n=== Processing artifact_img" + imgIdx + ".jpg (" + raw.cols() + "x" + raw.rows() + ") ===");
            String prefix = outputDir + "/img" + imgIdx;

            // Step 0: Save raw input
            Imgcodecs.imwrite(prefix + "_0_raw.jpg", raw);

            // Step 1: Letterbox to 1280x720 (preserve aspect ratio, pad with black)
            Mat resized = letterbox(raw, 1280, 720);
            Imgcodecs.imwrite(prefix + "_1_resized.jpg", resized);

            // Step 2: Convert to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

            // Step 3a: Green mask (raw threshold)
            Mat greenMaskRaw = new Mat();
            Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, greenMaskRaw);
            Imgcodecs.imwrite(prefix + "_2a_green_mask_raw.jpg", greenMaskRaw);

            // Step 3b: Purple mask (raw threshold)
            Mat purpleMaskRaw = new Mat();
            Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, purpleMaskRaw);
            Imgcodecs.imwrite(prefix + "_2b_purple_mask_raw.jpg", purpleMaskRaw);

            // Step 4: Morphological filtering (open + close)
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

            Mat greenMaskFiltered = greenMaskRaw.clone();
            Imgproc.morphologyEx(greenMaskFiltered, greenMaskFiltered, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(greenMaskFiltered, greenMaskFiltered, Imgproc.MORPH_CLOSE, kernel);
            Imgcodecs.imwrite(prefix + "_3a_green_mask_filtered.jpg", greenMaskFiltered);

            Mat purpleMaskFiltered = purpleMaskRaw.clone();
            Imgproc.morphologyEx(purpleMaskFiltered, purpleMaskFiltered, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(purpleMaskFiltered, purpleMaskFiltered, Imgproc.MORPH_CLOSE, kernel);
            Imgcodecs.imwrite(prefix + "_3b_purple_mask_filtered.jpg", purpleMaskFiltered);

            // Step 5: Find contours and filter by area + aspect ratio
            Mat contourVis = resized.clone();
            Mat finalVis = resized.clone();

            double totalPixels = resized.cols() * resized.rows();
            double minArea = MIN_AREA_FRACTION * totalPixels;
            double maxArea = MAX_AREA_FRACTION * totalPixels;

            processColor(contourVis, finalVis, greenMaskFiltered, "Green Ball",
                    new Scalar(0, 255, 0), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea, prefix, imgIdx);
            processColor(contourVis, finalVis, purpleMaskFiltered, "Purple Ball",
                    new Scalar(255, 0, 255), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea, prefix, imgIdx);

            Imgcodecs.imwrite(prefix + "_4_contours_all.jpg", contourVis);
            Imgcodecs.imwrite(prefix + "_5_final_detections.jpg", finalVis);

            // Cleanup
            raw.release(); resized.release(); hsv.release();
            greenMaskRaw.release(); purpleMaskRaw.release();
            greenMaskFiltered.release(); purpleMaskFiltered.release();
            kernel.release(); contourVis.release(); finalVis.release();
        }

        System.out.println("\n=== Pipeline visualization complete ===");
        System.out.println("Output saved to: " + outputDir);
    }

    /**
     * Scales the input to fit within targetW x targetH, then pads with black to reach exact size.
     */
    private static Mat letterbox(Mat input, int targetW, int targetH) {
        double scale = Math.min((double) targetW / input.cols(), (double) targetH / input.rows());
        int scaledW = (int) (input.cols() * scale);
        int scaledH = (int) (input.rows() * scale);

        Mat scaled = new Mat();
        Imgproc.resize(input, scaled, new Size(scaledW, scaledH));

        Mat output = Mat.zeros(targetH, targetW, input.type());
        scaled.copyTo(output.submat(0, scaledH, 0, scaledW));
        scaled.release();
        return output;
    }

    private static void processColor(Mat contourVis, Mat finalVis, Mat mask, String label,
                                     Scalar color, double fx, double fy, double cx, double cy,
                                     double cameraHeight, double minArea, double maxArea, String prefix, int imgIdx) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int accepted = 0, rejected = 0;
        Scalar rejectColor = new Scalar(100, 100, 100); // gray for rejected

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            double aspectRatio = (double) rect.width / rect.height;

            boolean areaOk = area > minArea && area < maxArea;
            boolean aspectOk = aspectRatio > MIN_ASPECT && aspectRatio < MAX_ASPECT;

            if (areaOk && aspectOk) {
                // Accepted contour
                accepted++;
                List<MatOfPoint> single = new ArrayList<>();
                single.add(contour);
                Imgproc.drawContours(contourVis, single, 0, color, 2);

                // Draw area label on contour vis
                Imgproc.putText(contourVis, String.format("A=%.0f AR=%.2f", area, aspectRatio),
                        new Point(rect.x, rect.y - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

                // Final detection: bounding box + position estimate
                Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                double nx = (center.x - cx) / fx;
                double ny = (center.y - cy) / fy;

                double relX = 0, relY = 0;
                if (ny > 0) {
                    relX = cameraHeight / ny;
                    relY = -nx * relX;
                }

                Imgproc.rectangle(finalVis, new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height), color, 3);
                Imgproc.circle(finalVis, center, 6, color, -1);

                String posLabel = String.format("%s %.0f\" fwd %.0f\" lat", label, relX, relY);
                Imgproc.putText(finalVis, posLabel, new Point(rect.x, rect.y - 8),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

                System.out.printf("  [%s] ACCEPT area=%.0f aspect=%.2f -> fwd=%.1f\" lat=%.1f\" px=(%.0f,%.0f)\n",
                        label, area, aspectRatio, relX, relY, center.x, center.y);
            } else {
                // Rejected contour
                rejected++;
                List<MatOfPoint> single = new ArrayList<>();
                single.add(contour);
                Imgproc.drawContours(contourVis, single, 0, rejectColor, 1);

                String reason = !areaOk ? String.format("area=%.0f", area) : String.format("AR=%.2f", aspectRatio);
                Imgproc.putText(contourVis, reason, new Point(rect.x, rect.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.35, rejectColor, 1);
            }
            contour.release();
        }
        hierarchy.release();
        System.out.printf("  %s: %d accepted, %d rejected\n", label, accepted, rejected);
    }
}
