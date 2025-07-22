package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvWebcam.StreamFormat;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp
@Config
public class PerspectiveSampleDetection extends LinearOpMode {
    OpenCvWebcam webcam;
    PerspectiveSampleDetectionPipeline pipeline;

    public static String WEBCAM_NAME = "Webcam 1";
    public static int CAMERA_WIDTH = 1920; // 1280;
    public static int CAMERA_HEIGHT = 1080; // 720;
    public static int WHITE_BALANCE_TEMPERATURE_BLUE = 4500;
    public static int WHITE_BALANCE_TEMPERATURE_RED = 4500;
    public static boolean AUTO_WHITE_BALANCE = true;
    public static int CAMERA_GAIN = 0;
    public static boolean AUTO_EXPOSURE = true;
    public static int CAMERA_EXPOSURE = 30;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Set camera controls
                GainControl gainControl = webcam.getGainControl();
                gainControl.setGain(CAMERA_GAIN);
                WhiteBalanceControl wbc = webcam.getWhiteBalanceControl();
                if (AUTO_WHITE_BALANCE) {
                    wbc.setMode(WhiteBalanceControl.Mode.AUTO);
                } else {
                    wbc.setMode(WhiteBalanceControl.Mode.MANUAL);
                    if (PerspectiveSampleDetectionPipeline.DETECT_BLUE) {
                        wbc.setWhiteBalanceTemperature(WHITE_BALANCE_TEMPERATURE_BLUE);
                    } else {
                        wbc.setWhiteBalanceTemperature(WHITE_BALANCE_TEMPERATURE_RED);
                    }
                }
                ExposureControl exposureControl = webcam.getExposureControl();
                if (AUTO_EXPOSURE) {
                    exposureControl.setMode(ExposureControl.Mode.Auto);
                } else {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(CAMERA_EXPOSURE, TimeUnit.MILLISECONDS);
                }

                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT,
                        OpenCvCameraRotation.UPRIGHT, StreamFormat.MJPEG);
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

        while (opModeIsActive()) {
            if (gamepad1.start) {
                PerspectiveSampleDetectionPipeline.DETECT_BLUE =
                    !PerspectiveSampleDetectionPipeline.DETECT_BLUE;
            }
            int whiteBalance = wbc.getWhiteBalanceTemperature();
            if (gamepad1.left_bumper){
                whiteBalance -= 10;
                wbc.setWhiteBalanceTemperature(whiteBalance);
            }
            if (gamepad1.right_bumper){
                whiteBalance += 10;
                wbc.setWhiteBalanceTemperature(whiteBalance);
            }
            int exposure = (int) exposureControl.getExposure(TimeUnit.MILLISECONDS);
            if (gamepad1.x) {
                exposure -= 1;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            }
            if (gamepad1.y) {
                exposure += 1;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            }
            if (gamepad1.a){
                telemetry.addData("Saving images.", "%b", true);
                // generate a file name based on the current time yyyyMMdd_HHmmss
                String filename = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
                ImageSaver.saveMatToDisk(pipeline.inputImage, filename + "_input");
                ImageSaver.saveMatToDisk(pipeline.undistortedImage, filename + "_undistorted");
                ImageSaver.saveMatToDisk(pipeline.warpedImage, filename + "_warped");
                ImageSaver.saveMatToDisk(pipeline.segmentedImage, filename + "_segmented");
            }

            telemetry.addData("Camera Exposure", "%d",
                (int) exposureControl.getExposure(TimeUnit.MILLISECONDS));
            telemetry.addData("Camera WBC", "%d %d %d",
                wbc.getWhiteBalanceTemperature(),
                wbc.getMinWhiteBalanceTemperature(),
                wbc.getMaxWhiteBalanceTemperature());
            for (int i = 0; i < pipeline.getDetections().size(); i++) {
                RotatedRect detection = pipeline.getDetections().get(i);
                telemetry.addData("Detection " + i, "%f %f %f %f %f",
                    detection.center.x, detection.center.y,
                    detection.size.width, detection.size.height,
                    detection.angle);
            }
            telemetry.update();
        }
    }
}