package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Arm Test", group="Linear OpMode")
public class ArmTest extends LinearOpMode {

    private Servo servo1 = null;
    private Servo servo2 = null;
    private Servo servo3 = null;
    private Servo servo4 = null;
    private Servo servo5 = null;
    private Servo servo6 = null;
    public static double SERVO_POS1 = 0.452;
    public static double SERVO_POS2 = 0.366;
    public static double SERVO_POS3 = 0.5;
    public static double SERVO_POS4 = 0.447;
    public static double SERVO_POS5 = 0.364;
    public static double SERVO_POS6 = 0.446;

    // Measurements in cm
    public final double turretHeight = 26.9;
    public final double arm1Length = 24.1;
    public final double arm2Length = 25.8;
    public final double wristLength = 9.1;
    // Height of highest camera point above tile
    public final double cameraHeight = 23.0;
    // How much the camera protrudes from turret axis of revolution
    public final double cameraLength = 7.4;
    public final double cameraWidth = 4.2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo1 = hardwareMap.get(Servo.class, "servo 1");
        servo2 = hardwareMap.get(Servo.class, "servo 2");
        servo3 = hardwareMap.get(Servo.class, "servo 3");
        servo4 = hardwareMap.get(Servo.class, "servo 4");
        servo5 = hardwareMap.get(Servo.class, "servo 5");
        servo6 = hardwareMap.get(Servo.class, "servo 6");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x) {
                SERVO_POS1 -= 0.0002;
            }
            if(gamepad1.y) {
                SERVO_POS1 += 0.0002;
            }
            if(gamepad1.a) {
                SERVO_POS2 -= 0.0002;
            }
            if(gamepad1.b) {
                SERVO_POS2 += 0.0002;
            }
            if (gamepad1.left_stick_x < 0) {
                SERVO_POS3 -= 0.0002;
            } else if (gamepad1.left_stick_x > 0) {
                SERVO_POS3 += 0.0002;
            }
            if (gamepad1.left_stick_y < 0) {
                SERVO_POS4 -= 0.0002;
            } else if (gamepad1.left_stick_y > 0) {
                SERVO_POS4 += 0.0002;
            }
            if (gamepad1.right_stick_x < 0) {
                SERVO_POS5 -= 0.0002;
            } else if (gamepad1.right_stick_x > 0) {
                SERVO_POS5 += 0.0002;
            }
            if (gamepad1.right_stick_y < 0) {
                SERVO_POS6 -= 0.0002;
            } else if (gamepad1.right_stick_y > 0) {
                SERVO_POS6 += 0.0002;
            }

            SERVO_POS1 = Range.clip(SERVO_POS1, 0, 1);
            SERVO_POS2 = Range.clip(SERVO_POS2, 0, 1);
            SERVO_POS3 = Range.clip(SERVO_POS3, 0, 1);
            SERVO_POS4 = Range.clip(SERVO_POS4, 0, 1);
            SERVO_POS5 = Range.clip(SERVO_POS5, 0, 1);
            SERVO_POS6 = Range.clip(SERVO_POS6, 0, 1);
            servo1.setPosition(SERVO_POS1);
            servo2.setPosition(SERVO_POS2);
            servo3.setPosition(SERVO_POS3);
            servo4.setPosition(SERVO_POS4);
            servo5.setPosition(SERVO_POS5);
            servo6.setPosition(SERVO_POS6);

            telemetry.addData("pos1", SERVO_POS1);
            telemetry.addData("pos2", SERVO_POS2);
            telemetry.addData("pos3", SERVO_POS3);
            telemetry.addData("pos4", SERVO_POS4);
            telemetry.addData("pos5", SERVO_POS5);
            telemetry.addData("pos6", SERVO_POS6);
            telemetry.update();
        }
    }
}
