package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Visualizes the v2 artifact detection pipeline (cosine similarity + inverse floodfill).
 */
public class ArtifactPipelineVisualizer2 {

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
        String baseDir = System.getProperty("user.dir");
        String imageDir = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision";
        String outputDir = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/pipeline2_output";

        new File(outputDir).mkdirs();

        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        VisionLocalizer.VisionPose3D cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;

        double fx = cameraMatrix.get(0, 0)[0];
        double fy = cameraMatrix.get(1, 1)[0];
        double cx = cameraMatrix.get(0, 2)[0];
        double cy = cameraMatrix.get(1, 2)[0];

        VisionArtifactDetector2.ColorTarget greenTarget =
                new VisionArtifactDetector2.ColorTarget("Green Ball", VisionArtifactDetector2.GREEN_REF_BGR, VisionArtifactDetector2.GREEN_THRESHOLD);
        VisionArtifactDetector2.ColorTarget purpleTarget =
                new VisionArtifactDetector2.ColorTarget("Purple Ball", VisionArtifactDetector2.PURPLE_REF_BGR, VisionArtifactDetector2.PURPLE_THRESHOLD);

        double minAreaFrac = 0.001;
        double maxAreaFrac = 0.04;

        for (int imgIdx = 1; imgIdx <= 5; imgIdx++) {
            String imagePath = imageDir + "/artifact_img" + imgIdx + ".jpg";
            Mat raw = Imgcodecs.imread(imagePath);
            if (raw.empty()) {
                System.err.println("Could not load: " + imagePath);
                continue;
            }
            System.out.println("\n=== Processing artifact_img" + imgIdx + ".jpg (" + raw.cols() + "x" + raw.rows() + ") ===");
            String prefix = outputDir + "/img" + imgIdx;

            // Step 0: Raw
            Imgcodecs.imwrite(prefix + "_0_raw.jpg", raw);

            // Step 1: Letterbox to 1280x720
            Mat resized = letterbox(raw, 1280, 720);
            Imgcodecs.imwrite(prefix + "_1_resized.jpg", resized);

            double totalPixels = resized.cols() * resized.rows();
            double minArea = minAreaFrac * totalPixels;
            double maxArea = maxAreaFrac * totalPixels;

            // Step 2: Cosine similarity heatmaps
            Mat greenSim = VisionArtifactDetector2.computeCosineSimilarity(resized, greenTarget);
            Mat purpleSim = VisionArtifactDetector2.computeCosineSimilarity(resized, purpleTarget);

            saveHeatmap(greenSim, prefix + "_2a_green_cosine.jpg");
            saveHeatmap(purpleSim, prefix + "_2b_purple_cosine.jpg");

            // Step 3: Convert to 8-bit + Gaussian blur
            Mat greenSim8 = new Mat(), purpleSim8 = new Mat();
            greenSim.convertTo(greenSim8, CvType.CV_8U, 255.0);
            purpleSim.convertTo(purpleSim8, CvType.CV_8U, 255.0);
            greenSim.release(); purpleSim.release();

            Size blurKernel = VisionArtifactDetector2.BLUR_KERNEL;
            Imgproc.GaussianBlur(greenSim8, greenSim8, blurKernel, 0);
            Imgproc.GaussianBlur(purpleSim8, purpleSim8, blurKernel, 0);

            Imgcodecs.imwrite(prefix + "_3a_green_blurred.jpg", greenSim8);
            Imgcodecs.imwrite(prefix + "_3b_purple_blurred.jpg", purpleSim8);

            // Step 4: Fixed per-color threshold
            Mat greenMask = new Mat(), purpleMask = new Mat();
            double greenThresh8 = greenTarget.threshold * 255;
            double purpleThresh8 = purpleTarget.threshold * 255;
            Imgproc.threshold(greenSim8, greenMask, greenThresh8, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(purpleSim8, purpleMask, purpleThresh8, 255, Imgproc.THRESH_BINARY);
            greenSim8.release(); purpleSim8.release();

            System.out.printf("  Fixed thresholds: green=%d/255 (%.2f), purple=%d/255 (%.2f)\n",
                    (int) greenThresh8, greenTarget.threshold, (int) purpleThresh8, purpleTarget.threshold);

            Imgcodecs.imwrite(prefix + "_4a_green_thresh.jpg", greenMask);
            Imgcodecs.imwrite(prefix + "_4b_purple_thresh.jpg", purpleMask);

            // Step 5: Morphological close to fill dark patches
            Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(11, 11));
            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_CLOSE, closeKernel);
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, closeKernel);
            closeKernel.release();

            Imgcodecs.imwrite(prefix + "_5a_green_closed.jpg", greenMask);
            Imgcodecs.imwrite(prefix + "_5b_purple_closed.jpg", purpleMask);

            // Step 6: Inverse floodfill
            VisionArtifactDetector2.inverseFloodFill(greenMask);
            VisionArtifactDetector2.inverseFloodFill(purpleMask);

            Imgcodecs.imwrite(prefix + "_6a_green_floodfill.jpg", greenMask);
            Imgcodecs.imwrite(prefix + "_6b_purple_floodfill.jpg", purpleMask);

            // Step 7: Contours + final detections
            Mat contourVis = resized.clone();
            Mat finalVis = resized.clone();

            processContours(contourVis, finalVis, greenMask, "Green Ball",
                    new Scalar(0, 255, 0), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea);
            processContours(contourVis, finalVis, purpleMask, "Purple Ball",
                    new Scalar(255, 0, 255), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea);

            Imgcodecs.imwrite(prefix + "_7_contours.jpg", contourVis);
            Imgcodecs.imwrite(prefix + "_8_final.jpg", finalVis);

            raw.release(); resized.release();
            greenMask.release(); purpleMask.release();
            contourVis.release(); finalVis.release();
        }

        System.out.println("\n=== Pipeline v2 visualization complete ===");
        System.out.println("Output saved to: " + outputDir);
    }

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

    private static void saveHeatmap(Mat floatMat, String path) {
        // Map [0, 1] -> [0, 255] with fixed range (not min-max normalized)
        Mat u8 = new Mat();
        floatMat.convertTo(u8, CvType.CV_8U, 255.0);
        Mat colormap = new Mat();
        Imgproc.applyColorMap(u8, colormap, Imgproc.COLORMAP_JET);
        Imgcodecs.imwrite(path, colormap);
        u8.release(); colormap.release();
    }

    private static void processContours(Mat contourVis, Mat finalVis, Mat mask, String label,
                                        Scalar color, double fx, double fy, double cx, double cy,
                                        double cameraHeight, double minArea, double maxArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Scalar rejectColor = new Scalar(100, 100, 100);
        int accepted = 0, rejected = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            double aspectRatio = (double) rect.width / rect.height;

            boolean areaOk = area > minArea && area < maxArea;

            if (areaOk) {
                accepted++;
                List<MatOfPoint> single = new ArrayList<>();
                single.add(contour);
                Imgproc.drawContours(contourVis, single, 0, color, 2);
                Imgproc.putText(contourVis, String.format("A=%.0f AR=%.2f", area, aspectRatio),
                        new Point(rect.x, rect.y - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

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
                rejected++;
                List<MatOfPoint> single = new ArrayList<>();
                single.add(contour);
                Imgproc.drawContours(contourVis, single, 0, rejectColor, 1);
                String reason = String.format("area=%.0f", area);
                Imgproc.putText(contourVis, reason, new Point(rect.x, rect.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.35, rejectColor, 1);
            }
            contour.release();
        }
        hierarchy.release();
        System.out.printf("  %s: %d accepted, %d rejected\n", label, accepted, rejected);
    }
}
