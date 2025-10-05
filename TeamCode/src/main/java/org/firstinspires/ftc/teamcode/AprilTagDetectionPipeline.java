package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.*;
import java.util.*;

public class AprilTagDetectionPipeline {
    // Camera calibration parameters
    private double fx, fy, cx, cy, cameraHeight, cameraAngleRad, tagHeight;

    public AprilTagDetectionPipeline(double fx, double fy, double cx, double cy, double cameraHeight, double cameraAngleRad, double tagHeight) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.cameraHeight = cameraHeight;
        this.cameraAngleRad = cameraAngleRad;
        this.tagHeight = tagHeight;
    }

    /**
     * Calculate 3D position of AprilTag from 2D pixel coordinates.
     * Equivalent to Python's calculate_target_position_3d.
     */
    public Point2d calculateTagPosition3D(double u_p, double v_p) {
        double x_cam = (u_p - cx) / fx;
        double y_cam = -(v_p - cy) / fy;
        double z_cam = 1.0;

        double cos_tilt = Math.cos(cameraAngleRad);
        double sin_tilt = Math.sin(cameraAngleRad);
        double x_world_dir = x_cam;
        double y_world_dir = y_cam * cos_tilt - z_cam * sin_tilt;
        double z_world_dir = y_cam * sin_tilt + z_cam * cos_tilt;

        if (Math.abs(y_world_dir) < 1e-9) {
            return new Point2d(0.0, 0.0);
        }
        double t = (tagHeight - cameraHeight) / y_world_dir;
        if (t <= 0) {
            return new Point2d(0.0, 0.0);
        }
        double x_world = t * x_world_dir;
        double z_world = t * z_world_dir;
        return new Point2d(x_world, z_world);
    }

    /**
     * AprilTag detection using ArUco is not supported in the standard OpenCV Android SDK.
     * Use EasyOpenCV's AprilTagDetectionPipeline for AprilTag detection on FTC robots.
     */
    // public List<Point> detectAprilTags(Mat image) { ... } // Removed unsupported code

    // Helper class for 2D points
    public static class Point2d {
        public double x, y;
        public Point2d(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
