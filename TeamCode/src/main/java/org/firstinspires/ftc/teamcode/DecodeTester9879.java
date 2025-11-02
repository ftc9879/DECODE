package org.firstinspires.ftc.teamcode;

/**
 * ================================================================================
 * DECODE TESTER 9879 - CONTROL MAPPING AND HARDWARE CONFIGURATION
 * ================================================================================
 *
 * GAMEPAD A (gamepad1) CONTROLS:
 * ------------------------------
 * Left Stick Y       -> Drive forward/backward (mecanum drive)
 * Left Stick X       -> Strafe left/right (mecanum drive)
 * Right Stick X      -> Rotation (mecanum drive)
 * Button A           -> Toggle supporting motor HIGH position (target: 2000 encoder ticks)
 * Button B           -> Toggle supporting motor LOW position (target: 0 encoder ticks)
 *
 * GAMEPAD B (gamepad2) CONTROLS:
 * ------------------------------
 * Button A           -> Toggle intake motor FORWARD (power: 0.5)
 * Button B           -> Toggle intake motor REVERSE (power: -0.5)
 * Button X           -> Toggle transfer servo position (0.0 <-> 0.5)
 * Button Y           -> Toggle transfer servo position (0.0 <-> 0.5)
 * Left Bumper        -> Toggle spinner motor FORWARD (power: 0.5)
 * Right Bumper       -> Toggle spinner motor REVERSE (power: -0.5)
 * Left Trigger       -> STOP intake motor (clears both intake toggles)
 * Right Trigger      -> STOP spinner motor (clears both spinner toggles)
 * D-pad Down         -> EMERGENCY STOP ALL (resets all toggles and stops all motors)
 *
 * HARDWARE PORT CONFIGURATION:
 * ----------------------------
 * NOTE: Port assignments are defined in the FTC Robot Controller configuration file.
 * The configuration names below must match those in your robot configuration:
 *
 * MOTORS (DcMotor):
 *   "drive_back_left"   -> Back left mecanum wheel (REVERSE direction)
 *   "drive_back_right"  -> Back right mecanum wheel (FORWARD direction)
 *   "drive_front_left"  -> Front left mecanum wheel (REVERSE direction)
 *   "drive_front_right" -> Front right mecanum wheel (FORWARD direction)
 *   "spinner"           -> Spinner mechanism motor (FORWARD direction)
 *   "intake"            -> Intake mechanism motor (FORWARD direction)
 *   "supporting"        -> Supporting arm motor (FORWARD direction, RUN_TO_POSITION mode)
 *
 * SERVOS:
 *   "transfer"          -> Transfer servo (range: 0.0 to 0.5)
 *
 * SENSORS:
 *   "color_sensor_1"    -> ColorSensor (first color sensor)
 *   "color_sensor_2"    -> ColorSensor (second color sensor)
 *
 * ODOMETRY:
 *   Odometry pods are assumed to connect via a pinpoint device (acknowledged but not used).
 *
 * ================================================================================
 */

// Checklist:
// - Outline hardware mapping and acknowledge odometry pods.
// - Implement mecanum drive with gamepad1 joysticks and safe motor handling.
// - Add toggle-debounced controls for intake, transfer servo, and supporting motor.
// - Provide telemetry and graceful error handling for missing hardware.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware Mapping Table:
 * drive_back_left   -> DcMotor
 * drive_back_right  -> DcMotor
 * drive_front_left  -> DcMotor
 * drive_front_right -> DcMotor
 * spinner           -> DcMotor
 * intake            -> DcMotor
 * supporting        -> DcMotor (RUN_TO_POSITION)
 * color_sensor_1    -> ColorSensor
 * color_sensor_2    -> ColorSensor
 * transfer          -> Servo
 *
 * Odometry pods are assumed to connect via a pinpoint device (not used here but acknowledged).
 */
@TeleOp(name = "9879 Decode Tester", group = "FTC")
public class DecodeTester9879 extends LinearOpMode {

    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor spinner;
    private DcMotor intake;
    private DcMotor supporting;
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private Servo transfer;

    // Toggle state holders with debounce tracking.
    private boolean intakePositiveActive = false;
    private boolean intakeNegativeActive = false;
    private boolean transferAtHigh = false;
    private boolean supportingHighActive = false;
    private boolean supportingLowActive = true;
    private boolean spinnerPositiveActive = false;
    private boolean spinnerNegativeActive = false;

    private boolean prevGamepadBA = false;
    private boolean prevGamepadBB = false;
    private boolean prevGamepadBX = false;
    private boolean prevGamepadBY = false;
    private boolean prevGamepadAA = false;
    private boolean prevGamepadAB = false;
    private boolean prevGamepadBLB = false;
    private boolean prevGamepadBRB = false;
    private boolean prevGamepadBLT = false;
    private boolean prevGamepadBDDDown = false;

    @Override
    public void runOpMode() {
        initHardwareSafely();
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        while (opModeIsActive()) {
            // --- Drive Control ---
            mecanumDrive();

            // --- Spinner Toggle Logic ---
            handleSpinnerToggles();

            // --- Intake Toggle Logic ---
            handleIntakeToggles();

            // --- Transfer Servo Toggle Logic ---
            handleTransferToggle();

            // --- Supporting Motor Toggle Logic ---
            handleSupportingToggles();

            // --- Global Stop Button Logic ---
            handleGlobalStopButton();

            // --- Telemetry for debugging and monitoring ---
            telemetry.addData("Intake Power", intake != null ? intake.getPower() : "missing");
            telemetry.addData("Transfer Position", transfer != null ? transfer.getPosition() : "missing");
            telemetry.addData("Supporting Target", supporting != null ? supporting.getTargetPosition() : "missing");
            telemetry.addData("Spinner Power", spinner != null ? spinner.getPower() : "missing");
            telemetry.addData("Color Sensor 1", colorSensor1 != null ? colorSensor1.argb() : "missing");
            telemetry.addData("Color Sensor 2", colorSensor2 != null ? colorSensor2.argb() : "missing");
            telemetry.update();
        }

        // Ensure everything is stopped when op mode ends.
        stopAll();
    }

    private void initHardwareSafely() {
        // Map motors with try/catch so the op mode can continue even if something is missing.
        driveBackLeft = getMotorSafely("drive_back_left", DcMotorSimple.Direction.REVERSE);
        driveBackRight = getMotorSafely("drive_back_right", DcMotorSimple.Direction.FORWARD);
        driveFrontLeft = getMotorSafely("drive_front_left", DcMotorSimple.Direction.REVERSE);
        driveFrontRight = getMotorSafely("drive_front_right", DcMotorSimple.Direction.FORWARD);
        spinner = getMotorSafely("spinner", DcMotorSimple.Direction.FORWARD);
        intake = getMotorSafely("intake", DcMotorSimple.Direction.FORWARD);
        supporting = getMotorSafely("supporting", DcMotorSimple.Direction.FORWARD);

        try {
            colorSensor1 = hardwareMap.get(ColorSensor.class, "color_sensor_1");
        } catch (Exception e) {
            telemetry.addLine("color_sensor_1 missing or failed to initialize");
            telemetry.log().add("color_sensor_1 missing: " + e.getMessage());
            colorSensor1 = null; // Keep running without sensor.
        }

        try {
            colorSensor2 = hardwareMap.get(ColorSensor.class, "color_sensor_2");
        } catch (Exception e) {
            telemetry.addLine("color_sensor_2 missing or failed to initialize");
            telemetry.log().add("color_sensor_2 missing: " + e.getMessage());
            colorSensor2 = null;
        }

        try {
            transfer = hardwareMap.get(Servo.class, "transfer");
            transfer.setPosition(0.0);
        } catch (Exception e) {
            telemetry.addLine("transfer servo missing or failed to initialize");
            telemetry.log().add("transfer missing: " + e.getMessage());
            transfer = null;
        }

        if (supporting != null) {
            // Configure supporting motor for encoder control.
            supporting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            supporting.setTargetPosition(0);
            supporting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            supporting.setPower(0.6); // Default power for RUN_TO_POSITION; will toggle targets later.

            // Ensure toggle tracking mirrors the initialized target.
            supportingHighActive = false;
            supportingLowActive = true;
            updateSupportingTargetFromToggles();
        }
    }

    private DcMotor getMotorSafely(String name, DcMotorSimple.Direction direction) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(direction);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
            return motor;
        } catch (Exception e) {
            telemetry.addLine(name + " motor missing or failed to initialize");
            telemetry.log().add(name + " missing: " + e.getMessage());
            return null;
        }
    }

    private void mecanumDrive() {
        // Read joystick inputs from gamepad1 (gamepad A per instructions).
        double y = gamepad1.left_stick_y;    // Forward/back.
        double x = gamepad1.left_stick_x;    // Strafe.
        double rx = gamepad1.right_stick_x;  // Rotation.

        // Invert Y because forward is negative on stick.
        double frontLeftPower = -y + x + rx;
        double backLeftPower = -y - x + rx;
        double frontRightPower = -y - x - rx;
        double backRightPower = -y + x - rx;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeftPower),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
                        )
                )
        );

        frontLeftPower /= max;
        backLeftPower /= max;
        frontRightPower /= max;
        backRightPower /= max;

        if (driveFrontLeft != null) {
            driveFrontLeft.setPower(frontLeftPower);
        }
        if (driveBackLeft != null) {
            driveBackLeft.setPower(backLeftPower);
        }
        if (driveFrontRight != null) {
            driveFrontRight.setPower(frontRightPower);
        }
        if (driveBackRight != null) {
            driveBackRight.setPower(backRightPower);
        }
    }

    private void handleIntakeToggles() {
        boolean currentA = gamepad2.a; // gamepad B A button.
        boolean currentB = gamepad2.b; // gamepad B B button.
        boolean triggerStopPressed = gamepad2.left_trigger > 0.5; // Analog trigger stop.

        // Toggle for positive intake.
        if (currentA && !prevGamepadBA) {
            if (intakePositiveActive) {
                intakePositiveActive = false;
            } else {
                intakePositiveActive = true;
                intakeNegativeActive = false; // Ensure only one direction is active.
            }
        }

        // Toggle for negative intake.
        if (currentB && !prevGamepadBB) {
            if (intakeNegativeActive) {
                intakeNegativeActive = false;
            } else {
                intakeNegativeActive = true;
                intakePositiveActive = false;
            }
        }

        // Debounced stop via left trigger clears both toggles.
        if (triggerStopPressed && !prevGamepadBLT) {
            intakePositiveActive = false;
            intakeNegativeActive = false;
        }

        prevGamepadBA = currentA;
        prevGamepadBB = currentB;
        prevGamepadBLT = triggerStopPressed;

        double targetPower = 0.0;
        if (intakePositiveActive) {
            targetPower = 0.5;
        } else if (intakeNegativeActive) {
            targetPower = -0.5;
        }

        if (intake != null) {
            intake.setPower(targetPower);
        }
    }

    private void handleTransferToggle() {
        boolean currentX = gamepad2.x;
        boolean currentY = gamepad2.y;

        boolean toggled = false;
        if (currentX && !prevGamepadBX) {
            toggled = true;
        }
        if (currentY && !prevGamepadBY) {
            toggled = true;
        }

        if (toggled) {
            transferAtHigh = !transferAtHigh;
            if (transfer != null) {
                transfer.setPosition(transferAtHigh ? 0.5 : 0.0);
            }
        }

        prevGamepadBX = currentX;
        prevGamepadBY = currentY;
    }

    private void handleSpinnerToggles() {
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;

        if (currentLB && !prevGamepadBLB) {
            if (spinnerPositiveActive) {
                spinnerPositiveActive = false;
            } else {
                spinnerPositiveActive = true;
                spinnerNegativeActive = false; // Only one direction should run at a time.
            }
        }

        if (currentRB && !prevGamepadBRB) {
            if (spinnerNegativeActive) {
                spinnerNegativeActive = false;
            } else {
                spinnerNegativeActive = true;
                spinnerPositiveActive = false;
            }
        }

        prevGamepadBLB = currentLB;
        prevGamepadBRB = currentRB;

        // Use gamepad2 right trigger as an immediate stop override.
        if (gamepad2.right_trigger > 0.5) {
            spinnerPositiveActive = false;
            spinnerNegativeActive = false;
        }

        double spinnerPower = 0.0;
        if (spinnerPositiveActive) {
            spinnerPower = 0.5;
        } else if (spinnerNegativeActive) {
            spinnerPower = -0.5;
        }

        if (spinner != null) {
            spinner.setPower(spinnerPower);
        }
    }

    private void handleGlobalStopButton() {
        boolean currentDpadDown = gamepad2.dpad_down; // Available button for emergency stop.

        if (currentDpadDown && !prevGamepadBDDDown) {
            // Clear toggles so motors remain stopped on subsequent loops.
            intakePositiveActive = false;
            intakeNegativeActive = false;
            spinnerPositiveActive = false;
            spinnerNegativeActive = false;
            supportingHighActive = false;
            supportingLowActive = true;
            transferAtHigh = false;

            updateSupportingTargetFromToggles();
            if (transfer != null) {
                transfer.setPosition(0.0);
            }

            stopAll();
        }

        prevGamepadBDDDown = currentDpadDown;
    }

    private void handleSupportingToggles() {
        boolean currentA = gamepad1.a; // gamepad A button A.
        boolean currentB = gamepad1.b; // gamepad A button B.

        if (currentA && !prevGamepadAA) {
            supportingHighActive = !supportingHighActive;
            if (supportingHighActive) {
                supportingLowActive = false; // Favor the high target when toggled on.
            } else {
                supportingLowActive = true; // Returning false means fall back to the low position.
            }
            updateSupportingTargetFromToggles();
        }

        if (currentB && !prevGamepadAB) {
            supportingLowActive = !supportingLowActive;
            if (supportingLowActive) {
                supportingHighActive = false; // Favor the low target when toggled on.
            } else {
                supportingHighActive = true; // When toggled back off, jump to the high position.
            }
            updateSupportingTargetFromToggles();
        }

        prevGamepadAA = currentA;
        prevGamepadAB = currentB;
    }

    private void updateSupportingTargetFromToggles() {
        if (supporting != null) {
            int target;
            if (supportingHighActive) {
                target = 2000;
            } else {
                // Either the low toggle is active or neither toggle is engaged; both lead to position 0.
                target = 0;
            }
            supporting.setTargetPosition(target);
            supporting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            supporting.setPower(0.6);
        }
    }

    private void stopAll() {
        // Reset toggled state holders so next activation starts from a known baseline.
        intakePositiveActive = false;
        intakeNegativeActive = false;
        spinnerPositiveActive = false;
        spinnerNegativeActive = false;
        supportingHighActive = false;
        supportingLowActive = true;
        transferAtHigh = false;

        updateSupportingTargetFromToggles();

        if (driveBackLeft != null) {
            driveBackLeft.setPower(0.0);
        }
        if (driveBackRight != null) {
            driveBackRight.setPower(0.0);
        }
        if (driveFrontLeft != null) {
            driveFrontLeft.setPower(0.0);
        }
        if (driveFrontRight != null) {
            driveFrontRight.setPower(0.0);
        }
        if (spinner != null) {
            spinner.setPower(0.0);
        }
        if (intake != null) {
            intake.setPower(0.0);
        }
        if (supporting != null) {
            supporting.setPower(0.0);
        }
        if (transfer != null) {
            transfer.setPosition(0.0);
        }
    }
}
