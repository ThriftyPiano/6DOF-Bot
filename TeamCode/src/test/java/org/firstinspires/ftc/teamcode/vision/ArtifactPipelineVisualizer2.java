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
        String imageDir = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/artifact_detection/images";
        String outputBase = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/artifact_detection/pipeline2_output";

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
            String imgOutputDir = outputBase + "/img" + imgIdx;
            new File(imgOutputDir).mkdirs();
            String prefix = imgOutputDir + "/img" + imgIdx;

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

            // Debug: save individual components for image 1 only
            if (imgIdx == 1) {
                Mat[] greenParts = VisionArtifactDetector2.computeCosineSimilarityComponents(resized, greenTarget);
                Mat[] purpleParts = VisionArtifactDetector2.computeCosineSimilarityComponents(resized, purpleTarget);
                saveHeatmap(greenParts[0], prefix + "_debug_green_cossim.jpg");
                saveHeatmap(greenParts[1], prefix + "_debug_green_satweight.jpg");
                saveHeatmap(purpleParts[0], prefix + "_debug_purple_cossim.jpg");
                saveHeatmap(purpleParts[1], prefix + "_debug_purple_satweight.jpg");
                for (Mat m : greenParts) m.release();
                for (Mat m : purpleParts) m.release();
            }

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

            // Step 7: Hough circle detection on floodfilled masks
            Mat houghVis = resized.clone();
            drawHoughCircles(houghVis, greenMask, "Green", new Scalar(0, 255, 0));
            drawHoughCircles(houghVis, purpleMask, "Purple", new Scalar(255, 0, 255));
            Imgcodecs.imwrite(prefix + "_7_hough.jpg", houghVis);
            houghVis.release();

            // Step 8: Contours + final detections
            Mat contourVis = resized.clone();
            Mat finalVis = resized.clone();

            processContours(contourVis, finalVis, greenMask, "Green Ball",
                    new Scalar(0, 255, 0), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea);
            processContours(contourVis, finalVis, purpleMask, "Purple Ball",
                    new Scalar(255, 0, 255), fx, fy, cx, cy, cameraTransform.z, minArea, maxArea);

            Imgcodecs.imwrite(prefix + "_8_contours.jpg", contourVis);
            Imgcodecs.imwrite(prefix + "_9_final.jpg", finalVis);

            raw.release(); resized.release();
            greenMask.release(); purpleMask.release();
            contourVis.release(); finalVis.release();
        }

        System.out.println("\n=== Pipeline v2 visualization complete ===");
        System.out.println("Output saved to: " + outputBase);
    }

    private static void drawHoughCircles(Mat vis, Mat mask, String label, Scalar color) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(mask, blurred, new Size(5, 5), 0);

        Mat circles = new Mat();
        Imgproc.HoughCircles(blurred, circles, Imgproc.HOUGH_GRADIENT,
                1.5, 40, 50, 30, 15, 120);
        blurred.release();

        List<double[]> raw = new ArrayList<>();
        for (int i = 0; i < circles.cols(); i++) {
            raw.add(circles.get(0, i));
        }
        circles.release();

        // Filter: only keep circles that are mostly filled with positive (white) pixels
        List<double[]> positive = new ArrayList<>();
        for (double[] c : raw) {
            int cx = (int) Math.round(c[0]);
            int cy = (int) Math.round(c[1]);
            int r = (int) Math.round(c[2]);

            // Count white pixels inside the circle
            int whiteCount = 0, totalCount = 0;
            int x0 = Math.max(0, cx - r), x1 = Math.min(mask.cols() - 1, cx + r);
            int y0 = Math.max(0, cy - r), y1 = Math.min(mask.rows() - 1, cy + r);
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r) {
                        totalCount++;
                        if (mask.get(y, x)[0] > 0) whiteCount++;
                    }
                }
            }
            double fillRatio = totalCount > 0 ? (double) whiteCount / totalCount : 0;
            if (fillRatio > 0.5) {
                positive.add(c);
            }
        }

        // Remove circles almost fully contained within a larger circle
        List<double[]> filtered = new ArrayList<>();
        for (int i = 0; i < positive.size(); i++) {
            double[] ci = positive.get(i);
            boolean contained = false;
            for (int j = 0; j < positive.size(); j++) {
                if (i == j) continue;
                double[] cj = positive.get(j);
                if (cj[2] <= ci[2]) continue;
                double dist = Math.sqrt(Math.pow(ci[0] - cj[0], 2) + Math.pow(ci[1] - cj[1], 2));
                if (dist + ci[2] < cj[2] * 1.2) {
                    contained = true;
                    break;
                }
            }
            if (!contained) filtered.add(ci);
        }

        // Merge overlapping circles of similar size
        // Use average center, radius = half the distance between intersection points
        List<double[]> merged = new ArrayList<>();
        boolean[] used = new boolean[filtered.size()];
        for (int i = 0; i < filtered.size(); i++) {
            if (used[i]) continue;
            double[] ci = filtered.get(i);
            List<double[]> group = new ArrayList<>();
            group.add(ci);
            for (int j = i + 1; j < filtered.size(); j++) {
                if (used[j]) continue;
                double[] cj = filtered.get(j);
                double dist = Math.sqrt(Math.pow(ci[0] - cj[0], 2) + Math.pow(ci[1] - cj[1], 2));
                double maxR = Math.max(ci[2], cj[2]);
                double minR = Math.min(ci[2], cj[2]);
                if (dist < maxR * 0.7 && minR > maxR * 0.5) {
                    group.add(cj);
                    used[j] = true;
                }
            }
            if (group.size() == 1) {
                merged.add(ci);
            } else {
                // Average center
                double sumX = 0, sumY = 0;
                for (double[] g : group) { sumX += g[0]; sumY += g[1]; }
                double avgX = sumX / group.size();
                double avgY = sumY / group.size();

                // Radius from intersection points of first two circles
                double[] c1 = group.get(0), c2 = group.get(1);
                double d = Math.sqrt(Math.pow(c1[0] - c2[0], 2) + Math.pow(c1[1] - c2[1], 2));
                double r1 = c1[2], r2 = c2[2];
                // Half-chord length: h = sqrt(r1^2 - a^2) where a = (d^2 + r1^2 - r2^2) / (2d)
                double a = (d * d + r1 * r1 - r2 * r2) / (2 * d);
                double hSq = r1 * r1 - a * a;
                double mergedR;
                if (hSq > 0) {
                    mergedR = Math.sqrt(hSq); // half the intersection chord distance
                } else {
                    // Circles don't actually intersect, fall back to average
                    double sumR = 0;
                    for (double[] g : group) sumR += g[2];
                    mergedR = sumR / group.size();
                }
                merged.add(new double[]{avgX, avgY, mergedR});
            }
        }

        System.out.printf("  [%s] Hough: %d raw -> %d positive -> %d filtered -> %d merged\n",
                label, raw.size(), positive.size(), filtered.size(), merged.size());

        for (double[] c : merged) {
            Point center = new Point(c[0], c[1]);
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(vis, center, radius, color, 2);
            Imgproc.circle(vis, center, 4, color, -1);
            Point bottom = new Point(center.x, center.y + radius);
            Imgproc.circle(vis, bottom, 4, new Scalar(0, 0, 255), -1);
            Imgproc.line(vis, center, bottom, new Scalar(0, 0, 255), 1);
            Imgproc.putText(vis, String.format("%s r=%d", label, radius),
                    new Point(center.x + radius + 5, center.y),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            System.out.printf("    center=(%.0f,%.0f) r=%d\n", c[0], c[1], radius);
        }
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
