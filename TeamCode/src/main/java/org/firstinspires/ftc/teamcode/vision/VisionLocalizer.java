package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * VisionLocalizer implements the MegaTag 2 algorithm for robot localization using AprilTags.
 */
public class VisionLocalizer {

    // Arducam Calibration Constants (1280x720 resolution)
    public static final double FX = 518.5498;
    public static final double FY = 516.8600;
    public static final double CX = 627.7407;
    public static final double CY = 375.1764;

    public static final double[] DIST_COEFFS = {
        -0.285759, 0.079079, -0.000350, 0.000320, -0.009423
    };

    // Camera transform: only Z (height) is used for now. 44cm = 17.32"
    public static final VisionPose3D ARDUCAM_TRANSFORM = new VisionPose3D(0, 0, 17.32, 0, 0, 0);

    // DECODE Season Field Constants (2025-2026) - Z is inches above floor
    public static final double TAG_SIZE_INCHES = 6.5; 
    public static final Map<Integer, VisionPose3D> DECODE_FIELD_MAP = new HashMap<Integer, VisionPose3D>() {{
        put(20, new VisionPose3D(-58.37, -55.64, 29.50, 0.0, 0.0, Math.toRadians(54.0)));
        put(24, new VisionPose3D(-58.37, 55.64, 29.50, 0.0, 0.0, Math.toRadians(-54.0)));
    }};

    private final Map<Integer, VisionPose3D> fieldMap;
    private final Mat cameraMatrix;
    private final MatOfDouble distCoeffs;
    private final VisionPose3D cameraTransform;

    public VisionLocalizer(Map<Integer, VisionPose3D> fieldMap, Mat cameraMatrix, MatOfDouble distCoeffs, 
                          VisionPose3D cameraTransform) {
        this.fieldMap = fieldMap;
        this.cameraMatrix = cameraMatrix;
        this.distCoeffs = distCoeffs;
        this.cameraTransform = cameraTransform;
    }

    public static Mat getArducamMatrix() {
        Mat matrix = new Mat(3, 3, CvType.CV_64F);
        matrix.put(0, 0, FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0);
        return matrix;
    }

    public static MatOfDouble getArducamDistCoeffs() {
        return new MatOfDouble(DIST_COEFFS);
    }

    public void prepareFrame(Mat input, Mat output) {
        Imgproc.resize(input, output, new Size(1280, 720));
    }

    /**
     * Estimates camera pose in field coordinates using solvePnP.
     * Averages across all detected tags.
     */
    public VisionPose3D estimateCameraPose(List<Detection> detections) {
        Map<Integer, VisionPose3D> perTag = estimateCameraPosePerTag(detections);
        if (perTag.isEmpty()) return null;

        double avgX = 0, avgY = 0, avgZ = 0;
        for (VisionPose3D p : perTag.values()) {
            avgX += p.x; avgY += p.y; avgZ += p.z;
        }
        int n = perTag.size();
        return new VisionPose3D(avgX / n, avgY / n, avgZ / n, 0, 0, 0);
    }

    /**
     * MegaTag2-style constrained pose estimation.
     * Uses camera heading in field coordinates + flat-on-floor assumption (roll=0, pitch=0)
     * to fix rotation, then solves for translation via linear least-squares across all visible tags.
     *
     * With rotation known, each observed 2D corner provides 2 equations in 3 unknowns (tx, ty, tz).
     * A single tag (4 corners) gives 8 equations for 3 unknowns — well over-determined.
     *
     * @param detections list of tag detections with corners in TL, TR, BR, BL order
     * @param headingRadians camera heading in field coordinates (radians, CCW positive from +X axis)
     * @return camera pose in field coordinates, or null if no known tags detected
     */
    public VisionPose3D estimateCameraPoseConstrained(List<Detection> detections, double headingRadians) {
        if (detections == null || detections.isEmpty()) return null;

        // Gather all 3D-2D correspondences across all visible known tags
        List<Point3> allObjectPoints = new ArrayList<>();
        List<Point> allImagePoints = new ArrayList<>();

        for (Detection det : detections) {
            if (!fieldMap.containsKey(det.id)) continue;
            VisionPose3D tagPose = fieldMap.get(det.id);
            double tagYaw = tagPose.yaw;
            double cosT = Math.cos(tagYaw), sinT = Math.sin(tagYaw);
            double half = TAG_SIZE_INCHES / 2.0;

            // Tag corners in tag-local frame (X-right, Y-down, Z-into-tag)
            double[][] tagLocal = {
                {-half, -half, 0}, // TL
                { half, -half, 0}, // TR
                { half,  half, 0}, // BR
                {-half,  half, 0}  // BL
            };

            // Transform to field coordinates
            // Tag axes in field: X_tag=(-sinYaw, cosYaw, 0), Y_tag=(0,0,-1), Z_tag=(-cosYaw,-sinYaw,0)
            for (int i = 0; i < 4; i++) {
                double lx = tagLocal[i][0], ly = tagLocal[i][1], lz = tagLocal[i][2];
                double fx = tagPose.x + (-sinT) * lx + (-cosT) * lz;
                double fy = tagPose.y + cosT * lx + (-sinT) * lz;
                double fz = tagPose.z + (-1) * ly;
                allObjectPoints.add(new Point3(fx, fy, fz));
                allImagePoints.add(new Point(det.corners[i].x, det.corners[i].y));
            }
        }

        if (allObjectPoints.isEmpty()) return null;

        // Build R_cam_field from heading (flat on floor: roll=0, pitch=0).
        //
        // Camera frame (OpenCV): X-right, Y-down, Z-forward.
        // Field frame: X/Y horizontal, Z-up.
        //
        // At heading h, camera faces direction (cos h, sin h, 0) in field.
        // Camera axes in field coordinates:
        //   Z_cam (forward) = (cos h, sin h, 0)
        //   Y_cam (down)    = (0, 0, -1)
        //   X_cam (right)   = Y_cam × Z_cam = (sin h, -cos h, 0)
        //
        // R_cam_field maps field vectors to camera frame. Its rows are the
        // camera axes expressed as field-coordinate vectors:
        //   Row 0 (X_cam): (sin h, -cos h,  0)
        //   Row 1 (Y_cam): (0,      0,     -1)
        //   Row 2 (Z_cam): (cos h,  sin h,  0)

        double ch = Math.cos(headingRadians), sh = Math.sin(headingRadians);

        double[][] R = {
            { sh, -ch, 0},
            {  0,   0, -1},
            { ch,  sh, 0}
        };

        // Solve for camera position t_field via linear least-squares.
        // P_cam = R*(P_field - t_field), let Q = R*P_field, T = R*t_field.
        // From projection equations, rearranging to eliminate division:
        //   fx*Tx - (u-cx)*Tz = fx*Qx - (u-cx)*Qz
        //   fy*Ty - (v-cy)*Tz = fy*Qy - (v-cy)*Qz
        // This is linear in T = [Tx, Ty, Tz]. Solve A*T = b, then t_field = R^T * T.

        // Undistort image points first
        MatOfPoint2f distorted = new MatOfPoint2f(allImagePoints.toArray(new Point[0]));
        MatOfPoint2f undistorted = new MatOfPoint2f();
        Calib3d.undistortPoints(distorted, undistorted, cameraMatrix, distCoeffs, new Mat(), cameraMatrix);
        Point[] undistPts = undistorted.toArray();

        int n = allObjectPoints.size();
        // A is (2n x 3), b is (2n x 1)
        Mat A = new Mat(2 * n, 3, CvType.CV_64F);
        Mat b = new Mat(2 * n, 1, CvType.CV_64F);

        double fxVal = cameraMatrix.get(0, 0)[0];
        double fyVal = cameraMatrix.get(1, 1)[0];
        double cxVal = cameraMatrix.get(0, 2)[0];
        double cyVal = cameraMatrix.get(1, 2)[0];

        for (int i = 0; i < n; i++) {
            Point3 pf = allObjectPoints.get(i);

            // Q = R * P_field
            double qx = R[0][0] * pf.x + R[0][1] * pf.y + R[0][2] * pf.z;
            double qy = R[1][0] * pf.x + R[1][1] * pf.y + R[1][2] * pf.z;
            double qz = R[2][0] * pf.x + R[2][1] * pf.y + R[2][2] * pf.z;

            double u = undistPts[i].x;
            double v = undistPts[i].y;

            // Row 2i: fx*Tx - (u-cx)*Tz = fx*Qx - (u-cx)*Qz
            A.put(2 * i, 0, fxVal);
            A.put(2 * i, 1, 0);
            A.put(2 * i, 2, -(u - cxVal));
            b.put(2 * i, 0, fxVal * qx - (u - cxVal) * qz);

            // Row 2i+1: fy*Ty - (v-cy)*Tz = fy*Qy - (v-cy)*Qz
            A.put(2 * i + 1, 0, 0);
            A.put(2 * i + 1, 1, fyVal);
            A.put(2 * i + 1, 2, -(v - cyVal));
            b.put(2 * i + 1, 0, fyVal * qy - (v - cyVal) * qz);
        }

        // Solve A * T = b via least squares (SVD)
        Mat T = new Mat();
        Core.solve(A, b, T, Core.DECOMP_SVD);

        // T = R * t_field  =>  t_field = R^T * T  (R is orthogonal so R^-1 = R^T)
        double Tx = T.get(0, 0)[0], Ty = T.get(1, 0)[0], Tz = T.get(2, 0)[0];
        double tfx = R[0][0] * Tx + R[1][0] * Ty + R[2][0] * Tz;
        double tfy = R[0][1] * Tx + R[1][1] * Ty + R[2][1] * Tz;
        double tfz = R[0][2] * Tx + R[1][2] * Ty + R[2][2] * Tz;

        // Clean up
        A.release(); b.release(); T.release();
        distorted.release(); undistorted.release();

        return new VisionPose3D(tfx, tfy, tfz, 0, 0, headingRadians);
    }

    /**
     * Returns a per-tag map of camera pose estimates in field coordinates.
     * Uses solvePnP to get tag-to-camera transform, then inverts to get camera position.
     */
    public Map<Integer, VisionPose3D> estimateCameraPosePerTag(List<Detection> detections) {
        Map<Integer, VisionPose3D> result = new HashMap<>();
        if (detections == null || detections.isEmpty()) return result;

        for (Detection detection : detections) {
            if (!fieldMap.containsKey(detection.id)) continue;

            VisionPose3D tagFieldPose = fieldMap.get(detection.id);
            double tagSize = TAG_SIZE_INCHES;

            // Tag corners in tag-local coordinates (Y-Down convention)
            // Expected corner order in Detection: TL, TR, BR, BL
            MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-tagSize/2, -tagSize/2, 0), // TL
                new Point3( tagSize/2, -tagSize/2, 0), // TR
                new Point3( tagSize/2,  tagSize/2, 0), // BR
                new Point3(-tagSize/2,  tagSize/2, 0)  // BL
            );

            MatOfPoint2f imagePoints = new MatOfPoint2f(detection.corners);
            Mat rvec = new Mat();
            Mat tvec = new Mat();

            if (Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec)) {
                // solvePnP gives us: tag position in camera frame (tvec) and rotation (rvec)
                // We need: camera position in tag frame, then transform to field frame

                // Convert rvec to rotation matrix R (tag-to-camera rotation)
                Mat R = new Mat();
                Calib3d.Rodrigues(rvec, R);

                // Camera position in tag frame = -R^T * tvec
                // R^T (3x3 transpose)
                double r00 = R.get(0,0)[0], r01 = R.get(0,1)[0], r02 = R.get(0,2)[0];
                double r10 = R.get(1,0)[0], r11 = R.get(1,1)[0], r12 = R.get(1,2)[0];
                double r20 = R.get(2,0)[0], r21 = R.get(2,1)[0], r22 = R.get(2,2)[0];

                double tx = tvec.get(0, 0)[0];
                double ty = tvec.get(1, 0)[0];
                double tz = tvec.get(2, 0)[0];

                // Camera in tag-local frame = -R^T * t
                double camInTagX = -(r00 * tx + r10 * ty + r20 * tz);
                double camInTagY = -(r01 * tx + r11 * ty + r21 * tz);
                double camInTagZ = -(r02 * tx + r12 * ty + r22 * tz);

                // Transform from tag-local to field coordinates
                // Tag local frame (OpenCV): X=right, Y=down, Z=into-tag (right-hand rule)
                // Field frame: X/Y horizontal plane, Z up
                // Tag yaw = direction the tag faces outward in field XY plane
                //
                // Tag axes in field coordinates:
                //   X_tag = (-sin(yaw), cos(yaw), 0)   [right when looking at tag]
                //   Y_tag = (0, 0, -1)                  [down = -Z_field]
                //   Z_tag = (-cos(yaw), -sin(yaw), 0)   [into wall, away from camera]
                double tagYaw = tagFieldPose.yaw;
                double cosTag = Math.cos(tagYaw);
                double sinTag = Math.sin(tagYaw);

                double camFieldX = tagFieldPose.x - sinTag * camInTagX - cosTag * camInTagZ;
                double camFieldY = tagFieldPose.y + cosTag * camInTagX - sinTag * camInTagZ;
                double camFieldZ = tagFieldPose.z - camInTagY;

                result.put(detection.id, new VisionPose3D(camFieldX, camFieldY, camFieldZ, 0, 0, 0));

                R.release();
            }
            rvec.release(); tvec.release(); objectPoints.release(); imagePoints.release();
        }

        return result;
    }

    private double[][] getRotationMatrix(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double[][] R = new double[3][3];
        R[0][0] = cy * cp;
        R[0][1] = cy * sp * sr - sy * cr;
        R[0][2] = cy * sp * cr + sy * sr;
        R[1][0] = sy * cp;
        R[1][1] = sy * sp * sr + cy * cr;
        R[1][2] = sy * sp * cr - cy * sr;
        R[2][0] = -sp;
        R[2][1] = cp * sr;
        R[2][2] = cp * cr;
        return R;
    }

    private double[] multiply(double[][] matrix, double[] vector) {
        double[] result = new double[3];
        for (int i = 0; i < 3; i++) result[i] = matrix[i][0] * vector[0] + matrix[i][1] * vector[1] + matrix[i][2] * vector[2];
        return result;
    }

    public static class VisionPose3D {
        public double x, y, z, roll, pitch, yaw;
        public VisionPose3D(double x, double y, double z, double roll, double pitch, double yaw) {
            this.x = x; this.y = y; this.z = z; this.roll = roll; this.pitch = pitch; this.yaw = yaw;
        }
        public String toString() { return String.format("Pose3D(x=%.2f, y=%.2f, z=%.2f, yaw=%.2f°)", x, y, z, Math.toDegrees(yaw)); }
    }

    /**
     * Detection with corners in order: TL, TR, BR, BL (looking at the tag face).
     * Callers must reorder corners from their detector's native order to this order.
     */
    public static class Detection {
        public int id; public Point[] corners;
        public Detection(int id, Point[] corners) { this.id = id; this.corners = corners; }
    }
}
