# Vision System

Vision processing for AprilTag-based camera localization and game element detection.

## Directory Structure

```
TeamCode/
  src/main/java/.../vision/
    VisionLocalizer.java        # Camera pose estimation via AprilTag solvePnP
    VisionArtifactDetector.java # Color segmentation + homography for game elements
  src/test/java/.../vision/
    VisionLocalizerImageTest.java  # Desktop test with captured field image
    capture_field.jpg              # Test image (camera at field origin, both tags visible)
    ...                            # Other test files
  libs/desktop/
    opencv-4.9.0-0.jar            # OpenCV Java bindings
    libopencv_java490_x64.so      # Native library (Linux x86_64)
```

## Running Tests Locally

### Prerequisites

- JDK installed (OpenJDK recommended)
- Linux x86_64 (native library provided in `libs/desktop/`)

### Compile and Run

From the **project root** (`ftc-robot/`):

```bash
# Compile source and test classes
javac -cp "TeamCode/libs/desktop/opencv-4.9.0-0.jar" \
    TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/VisionLocalizer.java \
    TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/VisionLocalizerImageTest.java \
    -d /tmp/vision_test_out

# Run the localizer image test
java -cp "/tmp/vision_test_out:TeamCode/libs/desktop/opencv-4.9.0-0.jar" \
    -Duser.dir="$(pwd)" \
    org.firstinspires.ftc.teamcode.vision.VisionLocalizerImageTest
```

Expected output: camera pose near (0, 0) with ~3" error.

## Build and Deploy to Robot

### Prerequisites

- Android SDK with `platform-tools` (adb)
- Robot Control Hub accessible at `ftcrobot.pystem.com:5555`

### Build APK

```bash
cd ftc-robot
./gradlew assembleDebug
```

### Connect and Install

```bash
export PATH="$PATH:$HOME/Android/Sdk/platform-tools"
adb connect ftcrobot.pystem.com:5555
adb -s ftcrobot.pystem.com:5555 install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

### Build + Deploy (one-liner)

```bash
./gradlew assembleDebug && \
    export PATH="$PATH:$HOME/Android/Sdk/platform-tools" && \
    adb connect ftcrobot.pystem.com:5555 && \
    adb -s ftcrobot.pystem.com:5555 install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

After install, wait ~15 seconds for the app to restart, then open FTC Dashboard at `https://ftcrobot.pystem.com/dash`.

## Key Notes

- Camera streams at 1920x1080 MJPEG; calibration is for 1280x720. VisionOpMode scales corners by 2/3.
- FTC SDK AprilTag corners are ordered BL, BR, TR, TL. VisionLocalizer expects TL, TR, BR, BL.
- OpenCV ArUco detector returns corners as BR, BL, TL, TR (different from FTC SDK).
- DECODE field map: Tag 20 at (-58.37, -55.64, 29.50) yaw=54deg, Tag 24 at (-58.37, 55.64, 29.50) yaw=-54deg.
