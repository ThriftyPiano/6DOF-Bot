package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.PerspectiveSampleDetection.AUTO_EXPOSURE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.TurretArm;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="TurretArmOpMode", group="Linear OpMode")
public class TurretArmOpMode extends LinearOpMode {
    // Servo Names for Telemetry
    private static final String[] SERVO_NAMES = new String[]{
        "Turret", "Arm1", "Arm2", "Wrist1", "Wrist2", "Claw"
    };
    private static String[] servoHardwareNames = new String[]{"servo 1", "servo 2", "servo 6", "servo 3", "servo 4", "servo 5"};

    OpenCvWebcam webcam;
    PerspectiveSampleDetectionPipeline pipeline;

    public static String WEBCAM_NAME = "Webcam 1";
    public static int CAMERA_WIDTH = 1920; // 1280;
    public static int CAMERA_HEIGHT = 1080; // 720;
    public static int WHITE_BALANCE_TEMPERATURE_BLUE = 4500;
    public static int WHITE_BALANCE_TEMPERATURE_RED = 4500;
    public static int CAMERA_GAIN = 0;
    public static int CAMERA_EXPOSURE = 30;

    public double[] xyzPos = new double[]{0, 40, 5};
    public double wristAngle = 0;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT,
                        OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                pipeline = new PerspectiveSampleDetectionPipeline(hardwareMap.appContext, telemetry);
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error appropriately
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        GainControl gainControl = webcam.getGainControl();
        WhiteBalanceControl wbc = webcam.getWhiteBalanceControl();
        ExposureControl exposureControl = webcam.getExposureControl();

        TurretArm turretArm = new TurretArm(hardwareMap, telemetry, servoHardwareNames);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean xyzUpdatedByDpadOrStick = false;
            double xDelta = 0;
            double yDelta = 0;
            double zDelta = 0;

            if (gamepad1.dpad_up) {
                yDelta += 0.0025;
            }
            if (gamepad1.dpad_down) {
                yDelta -= 0.0025;
            }
            if (gamepad1.dpad_left) {
                xDelta -= 0.0025;
            }
            if (gamepad1.dpad_right) {
                xDelta += 0.0025;
            }

            float stickY = gamepad1.left_stick_y;
            // Apply deadzone to left_stick_y
            if (Math.abs(stickY) > 0.1) {
                if (stickY > 0) { // Stick pushed down (positive Y value)
                    zDelta -= 0.0025;
                } else { // Stick pushed up (negative Y value, stickY < -0.1)
                    zDelta += 0.0025;
                }
            }

            if (xDelta != 0 || yDelta != 0 || zDelta != 0) {
                xyzPos[0] += xDelta;
                xyzPos[1] += yDelta;
                xyzPos[2] += zDelta;
            }

            List<RotatedRect> detections = pipeline.getDetections();

            for (int i = 0; i < detections.size(); i++) {
                RotatedRect detection = detections.get(i);
                telemetry.addData("Detection " + i, "%f %f %f %f %f",
                    detection.center.x, detection.center.y,
                    detection.size.width, detection.size.height,
                    detection.angle);
            }

            if (gamepad1.a & !detections.isEmpty()) {
                RotatedRect detection = detections.get(0);
                xyzPos[0] = detection.center.x / 10 - 1;
                xyzPos[1] = -detection.center.y / 10 + 6.5;
                xyzPos[2] = 3.5;
                wristAngle = detection.angle + Math.toDegrees(Math.atan2(xyzPos[1], xyzPos[0])) - 90;
            }
            if (gamepad1.right_stick_button) {
                turretArm.setServoPosXYZ(xyzPos, 2);
                turretArm.setSingleServoDegrees(wristAngle, 4, -1);
                turretArm.openClaw(-1);
            }
            if (gamepad1.b) {
                turretArm.setServoAnglesDegrees(new double[]{0, 150, 120, -90, 0, 90}, 2);
            }
            if (gamepad1.x) {
                turretArm.openClaw(0.5);
            }
            if (gamepad1.y) {
                turretArm.closeClaw(0.5);
            }
            if (gamepad1.left_stick_button) {
                turretArm.setServoAnglesDegrees(new double[]{0, 90, 0, -90, 0, 90}, 2);
            }

            turretArm.run();

            // Telemetry
            telemetry.addData("XYZ Coords", "X: %.2f, Y: %.2f, Z: %.2f", xyzPos[0], xyzPos[1], xyzPos[2]);
            telemetry.addData("Wrist Angle", "%.2f", wristAngle);
            telemetry.update();
        }
    }
}
