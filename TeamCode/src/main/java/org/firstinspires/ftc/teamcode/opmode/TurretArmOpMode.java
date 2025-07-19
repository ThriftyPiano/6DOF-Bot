package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TurretArm;


@TeleOp(name="TurretArmOpMode", group="Linear OpMode")
public class TurretArmOpMode extends LinearOpMode {
    // Servo Names for Telemetry
    private static final String[] SERVO_NAMES = new String[]{
        "Turret", "Arm1", "Arm2", "Wrist1", "Wrist2", "Claw"
    };
    private static String[] servoHardwareNames = new String[]{"servo 1", "servo 2", "servo 6", "servo 3", "servo 4", "servo 5"};


    public double[] xyzPos = new double[]{0, 40, 5};

    @Override
    public void runOpMode() {
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

            if (gamepad1.a) {
                turretArm.setServoPosXYZ(xyzPos, 0.5);
            }
            if (gamepad1.b) {
                turretArm.setServoAnglesDegrees(new double[]{0, 150, 120, -90, 0, 90}, 1);
            }
            if (gamepad1.x) {
                turretArm.openClaw();
            }
            if (gamepad1.y) {
                turretArm.closeClaw();
            }
            if (gamepad1.left_bumper) {
                turretArm.setSingleServoDegrees(-45, 4,0.5);
            }
            if (gamepad1.right_bumper) {
                turretArm.setSingleServoDegrees(45, 4,0.5);
            }
            if (gamepad1.left_stick_button) {
                turretArm.setServoAnglesDegrees(new double[]{0, 90, 0, -90, 0, 90}, 1);
            }

            turretArm.run();

            // Telemetry
            telemetry.addData("XYZ Coords", "X: %.2f, Y: %.2f, Z: %.2f", xyzPos[0], xyzPos[1], xyzPos[2]);
            telemetry.update();
        }
    }
}
