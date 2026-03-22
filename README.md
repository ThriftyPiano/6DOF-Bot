# FTC Robot Controller
*Computer Vision & Multi-DOF Inverse Kinematics*

---

## Overview

This FTC Robot Controller combines machine learning-based computer vision, mathematical inverse kinematics, and real-time control systems for competitive robotics. Built for FIRST Tech Challenge (FTC), the project uses mathematics and ML techniques to solve manual tasks that are normally done with human practice.

### Key Features
- **YOLOv8 Integration**: Fine-tuned YOLO model for FTC game element recognition 
- **Homography**: OpenCV perspective transform for easier object detection
- **6-DOF Inverse Kinematics**: Mathematical algorithms for precise robotic arm control
- **Camera Calibration**: Custom calibration pipeline with perspective correction

---

## Project Impact & Results

### Technical Achievements - FTC 2024-25
- **Detection Accuracy**: >80% FTC game element recall in various lightings
- **Positional Precision**: Sub-centimeter accuracy for autonomous manipulation  
- **Response Time**: <600ms from detection to motion planning

### Technical Achievements - FTC 2025-26
- **Tag Detection**: >95% April Tag detection rate with wide-angle lens
- **Response Time**: <500ms detection time
- **Work In Progress**: Goal alignment with color segmentation and contour analysis

### Skills Demonstrated
- **Computer Vision**: Image preprocessing, neural network training and deployment, camera calibration
- **Mathematics**: Linear algebra, trigonometry, constrained IK solver
- **Software Engineering**: Object-oriented design, real-time systems, optimization

---

## Technical Challenges & Solutions

### 1. Reducing Localization Error with Constrained Pose Estimation

**Problem:** Even after distortion correction, unconstrained `solvePnP` still gave 3.03" positional error and unreliable heading estimates from a single camera view — not accurate enough for autonomous navigation.

**Solution:** Implemented a MegaTag2-style constrained solver that fuses the IMU yaw reading with AprilTag detections. By fixing the robot's heading (from IMU) and assuming the camera is level (roll = 0, pitch = 0), the 6-DOF pose problem reduces to solving for 3 translation unknowns. Each tag corner provides 2 linear equations, so a single tag (4 corners) yields an over-determined system of 8 equations solved via SVD least-squares. This brought localization error down to **1.88"** with consistent heading accuracy.

### 2. Deploying YOLOv8 on Mobile Hardware

**Problem:** Running a neural network for game element detection on an Android phone with real-time constraints (<600ms per frame). The model needed to detect oriented bounding boxes (OBB) for samples at varying angles.

**Solution:** Trained a YOLOv8 OBB model, quantized it to TFLite format, and loaded it via `MappedByteBuffer` for efficient Android inference. Post-processing involved parsing the raw output tensor (8 channels × 3150 grid cells), applying confidence thresholding (>0.2), and spatial deduplication to merge nearby detections. Input was resized to 320×480 to balance speed and accuracy within the real-time budget.

### 3. Perspective Homography for Sample Inspection

**Problem:** The camera views the sample inspection area at an angle, distorting the apparent position and orientation of game elements. Pixel coordinates don't directly correspond to physical positions on the table surface.

**Solution:** Computed a 4-point homography matrix mapping the angled camera view to a top-down "bird's eye" perspective. Calibrated three separate transformation matrices for different camera mounting positions. After warping, pixel coordinates map linearly to millimeter positions on the table, enabling accurate sample localization and grasp planning.

### 4. Projecting 2D Detections to Field Coordinates

**Problem:** The artifact detector identifies objects in pixel space, but autonomous navigation requires field-relative positions. Converting between these frames involves the camera's height, heading, and intrinsic parameters.

**Solution:** Built a ground-plane projection model: for each detected pixel below the horizon, computed the forward distance using `cameraHeight / tan(angle_below_horizon)` and the lateral offset from the normalized x-coordinate. Combined this with the robot's field pose (from AprilTag localization + IMU heading) to transform camera-relative positions into absolute field coordinates using a 2D rotation: `fieldX = poseX + relX·cos(heading) - relY·sin(heading)`.

### 5. Analytical 6-DOF Inverse Kinematics with Reach Safety

**Problem:** The robot arm has 6 servos (turret, two arm joints, two wrist joints, claw) and needs to reach arbitrary 3D positions. Out-of-reach targets cause `acos` to return `NaN`, crashing the control loop.

**Solution:** Implemented a 2-link analytical IK solver that computes turret angle via `atan2`, then solves the arm joint angles using the law of cosines in the vertical plane. Added a reach safety check that scales down the target position if it exceeds the arm's maximum span: `scaleFactor = min(1, maxReach / distance) - margin`. Clamped all `acos` inputs to [-1, 1] to prevent floating-point edge cases. Smooth motion was achieved through linear interpolation over time to avoid jerky servo movements.

### 6. Adaptive Camera Exposure Control

**Problem:** Match venue lighting varies dramatically between practice fields and competition stages, causing color detection to fail when exposure is fixed.

**Solution:** Implemented a closed-loop exposure controller: after each frame, computed the mean brightness of the warped image and compared it to a target value (80 for blue detection, 65 for red). The exposure adjusts gradually (`exposure -= error / 5`) to avoid oscillation. Added manual overrides via gamepad (gain, exposure, white balance) for quick tuning during matches.

### 7. Desktop Testing Without Robot Hardware

**Problem:** Iterating on vision algorithms required deploying to the robot, which meant physically connecting to hardware for every code change — extremely slow for development.

**Solution:** Built a parallel desktop testing infrastructure that loads native OpenCV libraries (arm64 for Apple Silicon, x64 for other platforms) and runs the same `VisionLocalizer` and `VisionArtifactDetector` classes on recorded video, image sequences, or live USB camera feeds. Tests replay IMU data from timestamped log files alongside video frames, enabling full sensor-fusion validation. Real-time field map visualization overlays estimated poses on the competition field image for visual verification.