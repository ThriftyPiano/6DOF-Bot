package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class AutoDriveTest extends LinearOpMode {
    // hardware
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor[] wheels = null;
    private IMU imu = null;
    private OpenCvWebcam webcam;

    // config
    private double speed = 0.4;
    static int imageWidth = 320;
    static int imageHeight = 240;

    // state
    private int foo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FPS", webcam.getFps());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            runSteps();
        }
    }

    private void initWebcam() {
        // Uncomment to see live preview on robot controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(
        //     hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(imageWidth, imageHeight,
                        OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    private void initHardware() {
        // Enable bulk reads auto mode.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        leftFront  = hardwareMap.get(DcMotor.class, "motor 4");
        leftBack  = hardwareMap.get(DcMotor.class, "motor 1");
        rightFront = hardwareMap.get(DcMotor.class, "motor 3");
        rightBack = hardwareMap.get(DcMotor.class, "motor 2");
        wheels = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        stopWheels();
        initWebcam();
    }

    private void stopWheels() {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].setPower(0);
        }
    }

    private void runSteps() throws InterruptedException {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(i / 2 == 0 ? speed : -speed);
        }
    }
}
