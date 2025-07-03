package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TurretArm", group="Linear OpMode")
public class TurretArm extends LinearOpMode {

    private ElapsedTime movementTimer = new ElapsedTime();
    public double prevTime;

    // Servo Hardware
    private Servo turretServo = null;
    private Servo arm1Servo = null;
    private Servo wristServo1 = null;
    private Servo wristServo2 = null;
    private Servo clawServo = null;
    private Servo arm2Servo = null;

    // Array of servo objects for indexed access
    private Servo[] servoArray;

    // Servo Names for Telemetry
    private static final String[] SERVO_NAMES = new String[]{
            "Turret", "Arm1", "Arm2", "Wrist1", "Wrist2", "Claw"
    };

    // Current target positions for servos (0.0 to 1.0)
    // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
    public static double[] servoInitialPosArray = new double[]{
            0.452, // Turret
            0.366, // Arm1
            0.446, // Arm2
            0.5,   // Wrist1
            0.447, // Wrist2
            0.364  // Claw
    };

    /***
     * Increasing servo position:
     * Turns turret counterclockwise when viewing from above
     * Moves arm1 up
     * Moves arm2 down
     * Moves wrist up
     * Turns wrist counterclockwise when viewing from front of claw
     * Closes claw
    */

    // Reference positions corresponding to 0-degree angle for each servo.
    // These values are based on the initial static position values used in TR calculations.
    // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
    private static final double[] SERVO_ZERO_DEG_POS_ARRAY = new double[]{
            0.452, // Turret
            0.366, // Arm1
            0.446, // Arm2
            0.5,   // Wrist1
            0.447, // Wrist2
            0.364  // Claw
    };

    // TR: turn rate => Change in angle (degrees) turned per servo position value (0.0 to 1.0)
    // Calculated in runOpMode after servos are initialized, using SERVO_ZERO_DEG_POS_ARRAY.
    // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
    public static double[] servoTRArray = new double[6];


    // Measurements in cm (can be made static if not modified, or instance if they vary per robot instance)
    public double turretHeight = 26.9;
    public double arm1Length = 24.1;
    public double arm2Length = 25.8;
    public double wristLength = 9.1;
    public double cameraHeight = 23.0; // Height of highest camera point above tile
    public double cameraLength = 7.4;  // How much the camera protrudes from turret axis of revolution
    public double cameraWidth = 4.2;

    public double timeElapsed = 0;
    public double totalMovementTime = 0;

    // Auxiliary position arrays for movement interpolation
    public double[] startPosArray = servoInitialPosArray.clone();
    public double[] endPosArray = servoInitialPosArray.clone();

    @Override
    public void runOpMode() {
        prevTime = movementTimer.seconds();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize Servos
        turretServo = hardwareMap.get(Servo.class, "servo 1");
        arm1Servo = hardwareMap.get(Servo.class, "servo 2");
        arm2Servo = hardwareMap.get(Servo.class, "servo 6"); // Mapped to servo 6 based on previous gamepad logic
        wristServo1 = hardwareMap.get(Servo.class, "servo 3");
        wristServo2 = hardwareMap.get(Servo.class, "servo 4");
        clawServo = hardwareMap.get(Servo.class, "servo 5");

        // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
        servoArray = new Servo[]{turretServo, arm1Servo, arm2Servo, wristServo1, wristServo2, clawServo};

        // Initialize Turn Rates (TR)
        // This calculation assumes the servo is at SERVO_ZERO_DEG_POS_ARRAY when the alternate position (for 90-degree turn) is measured.
        // If the servo is NOT at its zero-degree position for these measurements, the TR will be incorrect.
        servoTRArray[0] = 90 / (0.9982 - SERVO_ZERO_DEG_POS_ARRAY[0]); // Turret
        servoTRArray[1] = 90 / (0.8786 - SERVO_ZERO_DEG_POS_ARRAY[1]); // Arm1
        servoTRArray[2] = 90 / (0.9978 - SERVO_ZERO_DEG_POS_ARRAY[2]); // Arm2
        servoTRArray[3] = 90 / (1.0    - SERVO_ZERO_DEG_POS_ARRAY[3]); // Wrist1
        servoTRArray[4] = 90 / (1.0    - SERVO_ZERO_DEG_POS_ARRAY[4]); // Wrist2
        servoTRArray[5] = 90 / (1.0    - SERVO_ZERO_DEG_POS_ARRAY[5]); // Claw

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Example calls to setServoAnglesDegrees:
        // if (some_condition) {
        //    setServoAnglesDegrees(new double[]{0, 0, 0, 0, 0, 0}); // Move all to 0 degrees
        // }
        // if (some_other_condition) {
        //    setServoAnglesDegrees(new double[]{30, 45, -20, 0, 10, 90}); // Example target angles
        // }

        while (opModeIsActive()) {
            double currentTime = movementTimer.seconds();
            timeElapsed += (currentTime - prevTime);
            prevTime = currentTime;

            if (gamepad1.a) {
                setServoAnglesDegrees(new double[]{0, 0, 0, 0, 0, 0}, 1);
            }
            else if (gamepad1.b) {
                setServoAnglesDegrees(new double[]{30, 30, 30, 30, 30, 0}, 1);
            }
            else if (gamepad1.x) {
                setServoAnglesDegrees(new double[]{0, 5.768, 45.842, -49.926, 0, -10}, 1);
            }

            if (totalMovementTime > 0) {
                if (timeElapsed > totalMovementTime){
                    timeElapsed = totalMovementTime;
                }
                double[] targetPositions = interpolatePositions(startPosArray, endPosArray, timeElapsed / totalMovementTime);
                for (int i = 0; i < servoArray.length; i++) {
                    if (servoArray[i] != null) {
                        servoArray[i].setPosition(targetPositions[i]); // Command the servo
                    }
                }

                // Update telemetry for target (interpolated) positions
                for (int i = 0; i < targetPositions.length; i++) {
                    telemetry.addData("Target (" + SERVO_NAMES[i] + ")", "%.4f", targetPositions[i]);
                }
            }

            // Telemetry
            for (int i = 0; i < servoInitialPosArray.length; i++) {
                telemetry.addData("Pos (" + SERVO_NAMES[i] + ")", "%.4f", servoInitialPosArray[i]);
                telemetry.addData("Pos (" + SERVO_NAMES[i] + ")", "%.4f", endPosArray[i]);
            }
            // Add telemetry for elapsed time
            telemetry.addData("Elapsed Time", "%.2f", timeElapsed);
            // Add telemetry for movementTimer.seconds();
            telemetry.addData("Movement Timer", "%.2f", movementTimer.seconds());
            telemetry.update();
        }
    }

    public double[] interpolatePositions(double[] startAngles, double[] endAngles, double ratio) {
        double [] interpolatedPositions = new double[startAngles.length];
        for (int i = 0; i < startAngles.length; i++) {
            interpolatedPositions[i] = startAngles[i] + (endAngles[i] - startAngles[i]) * ratio;
        }
        return interpolatedPositions;
    }

    public void setServoAnglesDegrees(double[] anglesDeg, double movementTime) {
        totalMovementTime = movementTime;
        timeElapsed = 0;
        if (anglesDeg == null || anglesDeg.length != servoArray.length) {
            telemetry.addData("Error", "setServoAnglesDegrees: Invalid angles array provided.");
            telemetry.update();
            return;
        }

        // Set startPosArray to current servo position array
        for (int i = 0; i < servoArray.length; i++) {
            if (servoArray[i] != null) {
                startPosArray[i] = servoArray[i].getPosition();
            }
        }

        for (int i = 0; i < servoArray.length; i++) {
            if (servoTRArray[i] == 0) { // Avoid division by zero if TR is not set (should not happen after init)
                telemetry.addData("Error", "TR not set for servo " + SERVO_NAMES[i]);
                continue;
            }
            // Formula: targetPos = zeroDegreePosition + (angleInDegrees / TurnRate)
            double newPos = SERVO_ZERO_DEG_POS_ARRAY[i] + (anglesDeg[i] / servoTRArray[i]);
            endPosArray[i] = Range.clip(newPos, 0, 1); // Update the global static position array
        }
        telemetry.update(); // If you add telemetry, remember to update
    }
}
