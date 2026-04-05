package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Minimal v3 pipeline: cosine similarity → blur → threshold → close → floodfill → contours.
 * No circle fitting or position estimation — just rough blob detection.
 */
public class ArtifactPipelineVisualizer3 {

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
        String detectionBase = baseDir + "/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/artifact_detection";
        String imageDir = args.length > 0 ? args[0] : detectionBase + "/images";
        String outputBase = args.length > 1 ? args[1] : detectionBase + "/pipeline3_output";

        // Discover all image files in imageDir
        File[] imageFiles = new File(imageDir).listFiles((dir, name) ->
                name.endsWith(".jpg") || name.endsWith(".jpeg") || name.endsWith(".png"));
        if (imageFiles == null || imageFiles.length == 0) {
            System.err.println("No images found in: " + imageDir);
            return;
        }
        java.util.Arrays.sort(imageFiles);

        VisionArtifactDetector2.ColorTarget greenTarget =
                new VisionArtifactDetector2.ColorTarget("Green Ball", VisionArtifactDetector2.GREEN_REF_BGR, VisionArtifactDetector2.GREEN_THRESHOLD);
        VisionArtifactDetector2.ColorTarget purpleTarget =
                new VisionArtifactDetector2.ColorTarget("Purple Ball", VisionArtifactDetector2.PURPLE_REF_BGR, VisionArtifactDetector2.PURPLE_THRESHOLD);

        double minAreaFrac = 0.001;

        for (File imageFile : imageFiles) {
            String imageName = imageFile.getName().replaceAll("\\.[^.]+$", "");
            Mat raw = Imgcodecs.imread(imageFile.getAbsolutePath());
            if (raw.empty()) {
                System.err.println("Could not load: " + imageFile);
                continue;
            }
            System.out.println("\n=== Processing " + imageFile.getName() + " (" + raw.cols() + "x" + raw.rows() + ") ===");
            String imgOutputDir = outputBase + "/" + imageName;
            new File(imgOutputDir).mkdirs();
            String prefix = imgOutputDir + "/" + imageName;

            // Step 1: Letterbox to 1280x720
            Mat resized = letterbox(raw, 1280, 720);

            double totalPixels = resized.cols() * resized.rows();
            double minArea = minAreaFrac * totalPixels;
            int closeSize = Math.max(3, (int)(resized.cols() * 0.0086) | 1);

            // Step 2: Cosine similarity
            Mat greenSim = VisionArtifactDetector2.computeCosineSimilarity(resized, greenTarget);
            Mat purpleSim = VisionArtifactDetector2.computeCosineSimilarity(resized, purpleTarget);

            saveHeatmap(greenSim, prefix + "_2a_green_cosine.jpg");
            saveHeatmap(purpleSim, prefix + "_2b_purple_cosine.jpg");

            // Step 3: Gaussian blur
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
            Imgproc.threshold(greenSim8, greenMask, greenTarget.threshold * 255, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(purpleSim8, purpleMask, purpleTarget.threshold * 255, 255, Imgproc.THRESH_BINARY);
            greenSim8.release(); purpleSim8.release();

            Imgcodecs.imwrite(prefix + "_4a_green_thresh.jpg", greenMask);
            Imgcodecs.imwrite(prefix + "_4b_purple_thresh.jpg", purpleMask);

            // Step 5: Morphological close (resolution-agnostic)
            Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(closeSize, closeSize));
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
            Mat finalVis = resized.clone();
            drawContours(finalVis, greenMask, "Green", new Scalar(0, 255, 0), minArea);
            drawContours(finalVis, purpleMask, "Purple", new Scalar(255, 0, 255), minArea);
            Imgcodecs.imwrite(prefix + "_7_final.jpg", finalVis);

            raw.release(); resized.release();
            greenMask.release(); purpleMask.release();
            finalVis.release();
        }

        System.out.println("\n=== Pipeline v3 visualization complete ===");
        System.out.println("Output saved to: " + outputBase);
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
        Mat u8 = new Mat();
        floatMat.convertTo(u8, CvType.CV_8U, 255.0);
        Mat colormap = new Mat();
        Imgproc.applyColorMap(u8, colormap, Imgproc.COLORMAP_JET);
        Imgcodecs.imwrite(path, colormap);
        u8.release(); colormap.release();
    }

    private static void drawContours(Mat vis, Mat mask, String label, Scalar color,
                                     double minArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int accepted = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                accepted++;
                Rect rect = Imgproc.boundingRect(contour);
                Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

                List<MatOfPoint> single = new ArrayList<>();
                single.add(contour);
                Imgproc.drawContours(vis, single, 0, color, 2);
                Imgproc.circle(vis, center, 5, color, -1);
                Imgproc.putText(vis, String.format("%s A=%.0f", label, area),
                        new Point(rect.x, rect.y - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

                System.out.printf("  [%s] center=(%.0f,%.0f) area=%.0f\n", label, center.x, center.y, area);
            }
            contour.release();
        }
        hierarchy.release();
        System.out.printf("  %s: %d detected\n", label, accepted);
    }
}
