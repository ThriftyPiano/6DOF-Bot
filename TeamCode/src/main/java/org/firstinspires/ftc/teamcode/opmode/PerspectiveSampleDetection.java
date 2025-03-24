package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp
public class PerspectiveSampleDetection extends LinearOpMode {
    OpenCvWebcam webcam;
    PerspectiveSampleDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720,
                        OpenCvCameraRotation.UPRIGHT, StreamFormat.MJPEG);
                pipeline = new PerspectiveSampleDetectionPipeline(hardwareMap.appContext, telemetry, webcam, false);
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

        // Manual adjust camera controls to find optimal values.
        pipeline.setAutoAdjustCameraControls(false);

        while (opModeIsActive()) {
            if (gamepad1.start) {
                pipeline.setDetectBlue(!pipeline.getDetectBlue());
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
            int cameraGain = gainControl.getGain();
            if (gamepad1.x) {
                cameraGain -= 1;
                gainControl.setGain(cameraGain);
            }
            if (gamepad1.y) {
                cameraGain += 1;
                gainControl.setGain(cameraGain);
            }
            long exposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
            if (gamepad1.a) {
                exposure -= 1;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            }
            if (gamepad1.b) {
                exposure += 1;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            }

            telemetry.addData("Camera Gain", "%d %d %d", gainControl.getGain(),
                    gainControl.getMinGain(), gainControl.getMaxGain());
            telemetry.addData("Camera WBC", "%d %d %d",
                    wbc.getWhiteBalanceTemperature(),
                    wbc.getMinWhiteBalanceTemperature(),
                    wbc.getMaxWhiteBalanceTemperature());
            telemetry.addData("Camera Exposure", "%d %d %d",
                    (int) exposureControl.getExposure(TimeUnit.MILLISECONDS),
                    (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS),
                    (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS));
            telemetry.addData("Detections", pipeline.getDetections().size());
            telemetry.addData("Brightness", pipeline.getBrightness());
            telemetry.addData("DetectBlue", pipeline.getDetectBlue());
            telemetry.update();
        }
    }
}