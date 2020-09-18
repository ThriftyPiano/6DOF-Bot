package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

@TeleOp
public class TwoWheelCameraDrive extends LinearOpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private Servo cameraLRServo = null;
    private Servo cameraUDServo = null;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double cameraLRPos = 0.458;
    private double cameraUDPos = 0.669;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor  = hardwareMap.get(DcMotor.class, "motor 2");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "motor 1");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        cameraLRServo = hardwareMap.get(Servo.class, "camera lr");
        cameraUDServo = hardwareMap.get(Servo.class, "camera ud");
        initCamera();

        waitForStart();

        while (opModeIsActive()) {
            double leftFrontPower;
            double rightFrontPower;
            if (gamepad1.left_bumper) {
                leftFrontPower   = 0.7 * gamepad1.left_stick_y;
                rightFrontPower  = 0.7 * gamepad1.right_stick_y;
            } else {
                leftFrontPower   = 0.3 * gamepad1.left_stick_y;
                rightFrontPower  = 0.3 * gamepad1.right_stick_y;
            }
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);

            if(gamepad1.x) {
                cameraLRPos += 0.0002;
            }
            if(gamepad1.y) {
                cameraLRPos -= 0.0002;
            }
            if(gamepad1.a) {
                cameraUDPos += 0.0002;
            }
            if(gamepad1.b) {
                cameraUDPos -= 0.0002;
            }
            cameraLRPos = Range.clip(cameraLRPos, 0, 1);
            cameraUDPos = Range.clip(cameraUDPos, 0, 1);
            cameraLRServo.setPosition(cameraLRPos);
            cameraUDServo.setPosition(cameraUDPos);

            telemetry.addData("Left Front Power", leftFrontPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.addData("Camera LR Pos", cameraLRPos);
            telemetry.addData("Camera UD Pos", cameraUDPos);
            telemetry.update();
        }
    }

    private void initCamera() {
        final CameraStreamProcessor processor = new CameraStreamProcessor();
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(processor)
            .build();
        FtcDashboard.getInstance().startCameraStream(processor, 0);
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}
