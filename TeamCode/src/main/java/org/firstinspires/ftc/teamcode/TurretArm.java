package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretArm {

    // Array of servo objects for indexed access
    private Servo[] servoArray;

    // Servo Names for Telemetry
    private static final String[] servoNames = new String[]{
        "Turret", "Arm1", "Arm2", "Wrist1", "Wrist2", "Claw"
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

    // TR: turn rate => Change in angle (degrees) turned per servo position value (0.0 to 1.0)
    // Calculated in runOpMode after servos are initialized, using SERVO_ZERO_DEG_POS_ARRAY.
    // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
    public static double[] servoTRArray = new double[6];


    // Measurements in cm (can be made static if not modified, or instance if they vary per robot instance)
    public double turretHeight = 26.9;
    public double arm1Length = 24.1;
    public double arm2Length = 25.8;
    public double wristLength = 9.1;

    public double timeElapsed = 0;
    public double totalMovementTime = 0;
    private ElapsedTime movementTimer = new ElapsedTime();
    public double prevTime;

    // Current target positions for servos (0.0 to 1.0)
    // Order: Turret, Arm1, Arm2, Wrist1, Wrist2, Claw
    public static double[] servoZeroPosArray = new double[]{
            0.4656, // Turret
            0.4182, // Arm1
            0.4678, // Arm2
            0.5894,   // Wrist1
            0.447, // Wrist2
            0.0128  // Claw
    };
    private static final double[] servo90DegPosArray = new double[]{
            0.8036, // Turret
            0.7504, // Arm1
            0.8162, // Arm2
            1.1474, // Wrist1
            0.9858, // Wrist2
            0.2636  // Claw
    };

    // Auxiliary position arrays for movement interpolation
    public double[] startPosArray;
    public double[] endPosArray;

    private Telemetry telemetry;

    public TurretArm(HardwareMap hardwareMap, Telemetry telemetryVar, String[] servoHardwareNames) {
        servoArray = new Servo[servoHardwareNames.length];
        for (int i = 0; i < servoHardwareNames.length; i++) {
            servoArray[i] = hardwareMap.get(Servo.class, servoHardwareNames[i]);
        }
        startPosArray = servoZeroPosArray.clone();
        endPosArray = servoZeroPosArray.clone();
        for (int i = 0; i < servo90DegPosArray.length; i++) {
            servoTRArray[i] = 90 / (servo90DegPosArray[i] - servoZeroPosArray[i]);
        }
        telemetry = telemetryVar;
    }

    public void run() {
        timeElapsed += movementTimer.seconds() - prevTime;
        prevTime = movementTimer.seconds();
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

            // Stop sending PWM signals if movement is finished.
            if (timeElapsed == totalMovementTime) {
                totalMovementTime = 0;
            }

            // Update telemetry for target (interpolated) positions
            for (int i = 0; i < targetPositions.length; i++) {
                telemetry.addData("Target (" + servoNames[i] + ")", "%.4f", targetPositions[i]);
            }
        }

        // Telemetry
        for (int i = 0; i < servoArray.length; i++) {
            telemetry.addData(servoNames[i] + " Start Pos", "%.4f", startPosArray[i]);
            telemetry.addData(servoNames[i] + " End Pos", "%.4f", endPosArray[i]);
        }
        // Add telemetry for elapsed time
        telemetry.addData("Elapsed Time", "%.2f", timeElapsed);
    }

    // Relative position to turret base in x, y, z (height from ground)
    private double[] calculateAngles(double[] relativePosition) {
        double groundDistance = Math.sqrt(relativePosition[0] * relativePosition[0] + relativePosition[1] * relativePosition[1]);
        double hoverDistance = relativePosition[2];

        double p = turretHeight;
        double q = arm1Length;
        double r = arm2Length;
        double s = wristLength + hoverDistance;
        double d = groundDistance;
        // Height difference between turret top and wrist top (target point for arm kinematics)
        double h = p - s;

        double[] armAngles_deg = new double[6];

        // Turret Angle (degrees)
        // Calculated from atan2, converted to degrees, and adjusted by -90 degrees.
        armAngles_deg[0] = Math.toDegrees(Math.atan2(relativePosition[1], relativePosition[0])) - 90;

        // Arm1 Angle (degrees, absolute, relative to horizontal)
        // Intermediate arm1Angle_rad is kept in radians for use in arm2Angle calculation.
        double val_for_acos = (d * d + h * h + q * q - r * r) / (2 * q * Math.sqrt(d * d + h * h));
        val_for_acos = Math.max(-1.0, Math.min(1.0, val_for_acos)); // Clamp to avoid NaN from acos
        double arm1Angle_rad = - Math.atan2(h, d) + Math.acos(val_for_acos);
        armAngles_deg[1] = Math.toDegrees(arm1Angle_rad);

        // Arm2 Angle (degrees, absolute, relative to horizontal)
        // Calculated using arm1Angle_rad (in radians) for sin/cos, then result converted to degrees.
        // This formula determines the absolute angle arm2 needs to make with the horizontal
        // to point from its pivot (end of arm1) to the target.
        double arm2Angle_rad = Math.atan2(h + q * Math.sin(arm1Angle_rad), d - q * Math.cos(arm1Angle_rad)) + arm1Angle_rad;
        if (arm2Angle_rad < 0) {
            arm2Angle_rad += Math.PI;
        }
        armAngles_deg[2] = Math.toDegrees(arm2Angle_rad);

        // Wrist1 Angle (degrees)
        armAngles_deg[3] = - 90 + (Math.toDegrees(arm2Angle_rad) - Math.toDegrees(arm1Angle_rad));

        // Wrist2 Angle (degrees) - Placeholder, not calculated from XYZ
        armAngles_deg[4] = -1000;
        // Claw Angle (degrees) - Placeholder, not calculated from XYZ
        armAngles_deg[5] = -1000;

        return armAngles_deg;
    }

    private double[] interpolatePositions(double[] startPositions, double[] endPositions, double ratio) {
        double [] interpolatedPositions = new double[startPositions.length];
        for (int i = 0; i < startPositions.length; i++) {
            interpolatedPositions[i] = startPositions[i] + (endPositions[i] - startPositions[i]) * ratio;
        }
        return interpolatedPositions;
    }

    public void setServoAnglesDegrees(double[] anglesDeg, double movementTime) {
        totalMovementTime = movementTime;
        timeElapsed = 0;
        if (anglesDeg == null || anglesDeg.length != servoArray.length) {
            telemetry.addData("Error", "setServoAnglesDegrees: Invalid angles array provided.");
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
                telemetry.addData("Error", "TR not set for servo " + servoNames[i]);
                continue;
            }
            if (anglesDeg[i] < -999) {
                continue;
            }
            // Formula: targetPos = zeroDegreePosition + (angleInDegrees / TurnRate)
            double newPos = servoZeroPosArray[i] + (anglesDeg[i] / servoTRArray[i]);
            endPosArray[i] = Range.clip(newPos, 0, 1); // Update the global static position array
        }
    }

    public void setServoPosXYZ(double[] xyzPos, double movementTime) {
        double[] calculatedAngles = calculateAngles(xyzPos);
        setServoAnglesDegrees(calculatedAngles, movementTime);
        // Add telemetry for calculated angles
        for (int i = 0; i < calculatedAngles.length; i++) {
            telemetry.addData("Calculated (" + servoNames[i] + ")", "%.4f", calculatedAngles[i]);
        }
    }
    
    public void setSingleServoDegrees(double angleDeg, int servoIndex, double movementTime) {
        totalMovementTime = movementTime;
        timeElapsed = 0;

        // Set startPosArray to current servo position array
        for (int i = 0; i < servoArray.length; i++) {
            if (servoArray[i] != null) {
                startPosArray[i] = servoArray[i].getPosition();
            }
        }

        if (servoTRArray[servoIndex] == 0) { // Avoid division by zero if TR is not set (should not happen after init)
            telemetry.addData("Error", "TR not set for servo " + servoNames[servoIndex]);
        }
        // Formula: targetPos = zeroDegreePosition + (angleInDegrees / TurnRate)
        double newPos = servoZeroPosArray[servoIndex] + (angleDeg / servoTRArray[servoIndex]);
        endPosArray[servoIndex] = Range.clip(newPos, 0, 1); // Update the global static position array
    }

    public void openClaw() {
        setSingleServoDegrees(0, 5, 0.5);
    }
    public void closeClaw() {
        setSingleServoDegrees(90, 5, 0.5);
    }
}
