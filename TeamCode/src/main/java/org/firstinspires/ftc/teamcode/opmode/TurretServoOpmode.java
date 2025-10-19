package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Servo Test", group = "Test")
public class TurretServoOpmode extends LinearOpMode {
    private Servo turretServo;
    private double turretPosition = 0.5; // Start at middle position
    private final double SERVO_SPEED = 0.005; // Adjust this value to change servo turning speed

    @Override
    public void runOpMode() {
        // Initialize the servo
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(turretPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick input (using left stick X)
            double joystickInput = gamepad1.left_stick_x;

            // Update servo position based on joystick
            // Only update if joystick is moved significantly (deadband)
            if (Math.abs(joystickInput) > 0.1) {
                turretPosition += joystickInput * SERVO_SPEED;
                
                // Clamp position between 0 and 1
                turretPosition = Math.max(0, Math.min(1, turretPosition));
                
                // Set the new position
                turretServo.setPosition(turretPosition);
            }

            // Display current position in telemetry
            telemetry.addData("Servo Position", String.format("%.3f", turretPosition));
            telemetry.addData("Joystick Value", String.format("%.3f", joystickInput));
            telemetry.update();

            // Small sleep to prevent CPU overload
            sleep(10);
        }
    }
}