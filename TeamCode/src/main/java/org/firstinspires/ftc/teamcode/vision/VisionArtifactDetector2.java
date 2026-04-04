package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

/**
 * VisionArtifactDetector2 identifies game elements (artifacts) using cosine similarity
 * color matching and inverse floodfill for hole closing.
 */
public class VisionArtifactDetector2 {

    public static class Artifact {
        public String type;
        public Point pixelPoint;
        public Rect boundingBox;
        public double relX;
        public double relY;

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

    public static class ColorTarget {
        public String label;
        public double[] refBGR;
        public double refNorm;
        public double threshold; // 0-1 cosine similarity threshold

        public ColorTarget(String label, double[] refBGR, double threshold) {
            this.label = label;
            this.refBGR = refBGR;
            this.refNorm = Math.sqrt(refBGR[0] * refBGR[0] + refBGR[1] * refBGR[1] + refBGR[2] * refBGR[2]);
            this.threshold = threshold;
        }
    }

    // Reference colors in BGR
    public static final double[] GREEN_REF_BGR = {136, 178, 90}; // #5ab288
    public static final double[] PURPLE_REF_BGR = {150, 40, 130};
    public static final double GREEN_THRESHOLD = 0.45;
    public static final double PURPLE_THRESHOLD = 0.45;
    public static final Size BLUR_KERNEL = new Size(9, 9);

    // Area thresholds as fraction of total frame pixels (resolution-agnostic)
    private static final double MIN_AREA_FRACTION = 0.001; // ~920px at 1280x720
    private static final double MAX_AREA_FRACTION = 0.04; // ~36800px at 1280x720

    private final List<ColorTarget> colorTargets = new ArrayList<>();
    private Mat cameraMatrix;
    private VisionLocalizer.VisionPose3D cameraTransform;

    public VisionArtifactDetector2(Mat cameraMatrix, VisionLocalizer.VisionPose3D cameraTransform) {
        this.cameraMatrix = cameraMatrix;
        this.cameraTransform = cameraTransform;

        colorTargets.add(new ColorTarget("Green Ball", GREEN_REF_BGR, GREEN_THRESHOLD));
        colorTargets.add(new ColorTarget("Purple Ball", PURPLE_REF_BGR, PURPLE_THRESHOLD));
    }

    public List<Artifact> detect(Mat frame) {
        List<Artifact> results = new ArrayList<>();
        if (frame == null || frame.empty()) return results;

        double fx = cameraMatrix.get(0, 0)[0];
        double fy = cameraMatrix.get(1, 1)[0];
        double cx = cameraMatrix.get(0, 2)[0];
        double cy = cameraMatrix.get(1, 2)[0];

        double totalPixels = frame.cols() * frame.rows();
        double minArea = MIN_AREA_FRACTION * totalPixels;
        double maxArea = MAX_AREA_FRACTION * totalPixels;

        for (ColorTarget target : colorTargets) {
            // 1. Cosine similarity
            Mat similarity = computeCosineSimilarity(frame, target);

            // 2. Gaussian blur + Otsu adaptive threshold (excluding zero-padding)
            Mat sim8 = new Mat();
            similarity.convertTo(sim8, CvType.CV_8U, 255.0);
            similarity.release();

            Imgproc.GaussianBlur(sim8, sim8, BLUR_KERNEL, 0);

            Mat mask = new Mat();
            Imgproc.threshold(sim8, mask, target.threshold * 255, 255, Imgproc.THRESH_BINARY);
            sim8.release();

            // 3. Morphological close to fill dark patches on ball surfaces
            Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(11, 11));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);
            closeKernel.release();

            // 4. Inverse floodfill to close holes
            inverseFloodFill(mask);

            // 5. Find contours and filter
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > minArea && area < maxArea) {
                    Rect rect = Imgproc.boundingRect(contour);
                    Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

                    double nx = (center.x - cx) / fx;
                    double ny = (center.y - cy) / fy;

                    double relX = 0, relY = 0;
                    if (ny > 0) {
                        relX = cameraTransform.z / ny;
                        relY = -nx * relX;
                    }

                    results.add(new Artifact(target.label, center, rect, relX, relY));
                }
                contour.release();
            }
            mask.release();
            hierarchy.release();
        }
        return results;
    }

    /**
     * Computes saturation-weighted chrominance cosine similarity.
     *
     * score = cos_sim * min(pixChromaMag / refChromaMag, 1.0)
     *
     * Cosine similarity dominates the score (color direction match).
     * The saturation weight only penalizes desaturated pixels (gray tiles)
     * without boosting oversaturated ones (red wall), since it's capped at 1.0.
     *
     * Returns a CV_64F mat with values clamped to [0, 1].
     */
    public static Mat computeCosineSimilarity(Mat bgr, ColorTarget target) {
        List<Mat> channels = new ArrayList<>();
        Core.split(bgr, channels);

        Mat bf = new Mat(), gf = new Mat(), rf = new Mat();
        channels.get(0).convertTo(bf, CvType.CV_64F);
        channels.get(1).convertTo(gf, CvType.CV_64F);
        channels.get(2).convertTo(rf, CvType.CV_64F);
        for (Mat ch : channels) ch.release();

        // Subtract per-pixel channel mean to get chrominance
        Mat mean = new Mat();
        Core.add(bf, gf, mean);
        Core.add(mean, rf, mean);
        Core.multiply(mean, new Scalar(1.0 / 3.0), mean);
        Core.subtract(bf, mean, bf);
        Core.subtract(gf, mean, gf);
        Core.subtract(rf, mean, rf);
        mean.release();

        // Reference chrominance
        double refMean = (target.refBGR[0] + target.refBGR[1] + target.refBGR[2]) / 3.0;
        double rb = target.refBGR[0] - refMean;
        double rg = target.refBGR[1] - refMean;
        double rr = target.refBGR[2] - refMean;
        double refChromaNorm = Math.sqrt(rb * rb + rg * rg + rr * rr);

        // Dot product on chrominance
        Mat dot = new Mat(), temp = new Mat();
        Core.multiply(bf, new Scalar(rb), dot);
        Core.multiply(gf, new Scalar(rg), temp);
        Core.add(dot, temp, dot);
        Core.multiply(rf, new Scalar(rr), temp);
        Core.add(dot, temp, dot);

        // Pixel chrominance magnitude
        Mat pixMagSq = new Mat();
        Core.multiply(bf, bf, pixMagSq);
        Core.multiply(gf, gf, temp);
        Core.add(pixMagSq, temp, pixMagSq);
        Core.multiply(rf, rf, temp);
        Core.add(pixMagSq, temp, pixMagSq);
        Mat pixMag = new Mat();
        Core.sqrt(pixMagSq, pixMag);
        Core.add(pixMag, new Scalar(1e-6), pixMag);

        // cos_sim = dot / (pixMag * refChromaNorm)
        Mat cosSim = new Mat();
        Core.divide(dot, pixMag, cosSim);
        Core.multiply(cosSim, new Scalar(1.0 / refChromaNorm), cosSim);

        // saturation weight = min(pixMag / refChromaNorm, 1.0)
        Mat satWeight = new Mat();
        Core.multiply(pixMag, new Scalar(1.0 / refChromaNorm), satWeight);
        Imgproc.threshold(satWeight, satWeight, 1.0, 1.0, Imgproc.THRESH_TRUNC);

        // score = cos_sim * satWeight
        Mat result = new Mat();
        Core.multiply(cosSim, satWeight, result);

        // Clamp to [0, 1]
        Imgproc.threshold(result, result, 0, 0, Imgproc.THRESH_TOZERO);
        Imgproc.threshold(result, result, 1.0, 1.0, Imgproc.THRESH_TRUNC);

        bf.release(); gf.release(); rf.release();
        temp.release(); dot.release(); pixMagSq.release();
        pixMag.release(); cosSim.release(); satWeight.release();

        return result;
    }

    /**
     * Fills interior holes in a binary mask by flood-filling the background
     * from the border, then OR-ing the unfilled regions (holes) back in.
     */
    public static void inverseFloodFill(Mat mask) {
        Mat inverted = new Mat();
        Core.bitwise_not(mask, inverted);

        // Flood fill from (0,0) — turns background-connected white to black
        Mat floodMask = Mat.zeros(inverted.rows() + 2, inverted.cols() + 2, CvType.CV_8U);
        Imgproc.floodFill(inverted, floodMask, new Point(0, 0), new Scalar(0));

        // Remaining white pixels are interior holes; OR into original mask
        Core.bitwise_or(mask, inverted, mask);

        inverted.release();
        floodMask.release();
    }
}
