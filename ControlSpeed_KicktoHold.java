package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Flywheel KickHold Test (LED)", group = "Test")
public class FlywheelKickHoldTestLED extends LinearOpMode {

    // ===== Hardware =====
    private DcMotorEx mFW;
    private Servo led;

    // ===== Encoder =====
    private static final double TICKS_PER_REV = 28.0;

    // ===== Quick tuning knobs =====
    // If you see overshoot/hunting after the kick: reduce FW_KICK_MS (e.g., 300 → 220).
    // If you get hub brownouts / voltage dip: reduce FW_KICK_POWER (1.0 → 0.85).
    private static final long FW_KICK_MS = 300;        // 250–400 ms typical
    private static final double FW_KICK_POWER = 1.0;   // 0.8–1.0 if brownout happens

    // ---------------- LED light -------------------
    private static final double LED_GREEN_POS = 0.5;   // Green
    private static final double LED_RED_POS = 0.277;   // Red (optional)
    private static final double LED_OFF_POS = 0.0;     // off (try 1 if 0 doesn't work)
    private static final double LED_RPM_TOL = 150;     // speed tolerance

    // ===== Flywheel state =====
    private final ElapsedTime fwKickTimer = new ElapsedTime();
    private boolean fwKickActive = false;

    private boolean flywheelOn = false;
    private double flywheelTargetRPM = 4500;

    // RPM presets (edit as you like)
    private static final double RPM_LOW = 3900;
    private static final double RPM_MID = 4500;
    private static final double RPM_HIGH = 4800;

    // Button edge detection
    private boolean lastY = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;

    @Override
    public void runOpMode() {

        // Map hardware
        mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        led = hardwareMap.get(Servo.class, "led");

        // Initial setup
        led.setPosition(LED_OFF_POS);

        mFW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Flywheel KickHold Test (LED) ready");
        telemetry.addLine("gamepad2 Y = toggle ON/OFF");
        telemetry.addLine("gamepad2 dpad_left=3900, dpad_down=4500, dpad_up=4800");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== RPM preset buttons =====
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;
            boolean left = gamepad2.dpad_left;

            if (up && !lastUp) {
                flywheelTargetRPM = RPM_HIGH;
                if (flywheelOn) startFlywheelKick(); // re-kick on setpoint change
            }
            if (down && !lastDown) {
                flywheelTargetRPM = RPM_MID;
                if (flywheelOn) startFlywheelKick();
            }
            if (left && !lastLeft) {
                flywheelTargetRPM = RPM_LOW;
                if (flywheelOn) startFlywheelKick();
            }

            lastUp = up;
            lastDown = down;
            lastLeft = left;

            // ===== Toggle flywheel ON/OFF with gamepad2 Y =====
            boolean y = gamepad2.y;
            if (y && !lastY) {
                flywheelOn = !flywheelOn;

                if (flywheelOn) {
                    startFlywheelKick();
                } else {
                    fwKickActive = false;
                    mFW.setPower(0);
                    led.setPosition(LED_OFF_POS);
                }
            }
            lastY = y;

            // ===== Apply kick-hold behavior =====
            applyFlywheelKickHold(flywheelOn, flywheelTargetRPM);

            // ===== Telemetry + LED =====
            double currentRPM = getFlywheelRPM();
            boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;

            if (!flywheelOn) {
                led.setPosition(LED_OFF_POS);
            } else {
                led.setPosition(atSpeed ? LED_GREEN_POS : LED_RED_POS);
            }

            telemetry.addData("FlywheelOn", flywheelOn);
            telemetry.addData("Phase", (flywheelOn ? (fwKickActive ? "KICK" : "HOLD") : "OFF"));
            telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
            telemetry.addData("CurrentRPM", "%.0f", currentRPM);
            telemetry.addData("AtSpeed", atSpeed ? "YES" : "NO");
            telemetry.addData("Mode", mFW.getMode());
            telemetry.update();
        }
    }

    // =========================================================
    // Main apply function
    // =========================================================
    private void applyFlywheelKickHold(boolean on, double targetRPM) {
        if (!on) {
            fwKickActive = false;
            mFW.setPower(0.0);
            return;
        }
        updateFlywheelKickHold(targetRPM);
    }

    // =========================================================
    // RPM helper
    // =========================================================
    private double getFlywheelRPM() {
        return mFW.getVelocity() * 60.0 / TICKS_PER_REV; // velocity is ticks/sec
    }

    // =========================================================
    // Start kick
    // =========================================================
    private void startFlywheelKick() {
        fwKickActive = true;
        fwKickTimer.reset();

        mFW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (mFW.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        mFW.setPower(FW_KICK_POWER);
    }

    // =========================================================
    // Update kick-hold each loop
    // =========================================================
    private boolean updateFlywheelKickHold(double targetRPM) {
        double targetTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;

        // Kick phase
        if (fwKickActive && fwKickTimer.milliseconds() < FW_KICK_MS) {
            if (mFW.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            mFW.setPower(FW_KICK_POWER);
            return true;
        }

        // Hold phase
        fwKickActive = false;
        if (mFW.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        mFW.setVelocity(targetTicksPerSec);
        return false;
    }
}



// ************************************************************************************

/* 
//*************Quick tuning knobs************
//If you see overshoot/hunting after the kick: reduce FW_KICK_MS (e.g., 300 → 220).
//If you get hub brownouts / voltage dip: reduce FW_KICK_POWER (1.0 → 0.85).

TICKS_PER_REV=28;

// Kick-then-hold state
private final ElapsedTime fwKickTimer = new ElapsedTime();
private boolean fwKickActive = false;

// Tune these two only
private static final long FW_KICK_MS = 300;      // 250–400 ms typical
private static final double FW_KICK_POWER = 1.0; // 0.8–1.0 if brownout happens


// =========================================================
// FLYWHEEL CONTROL (toggle gamepad2 Y)
// =========================================================
private void applyFlywheelControl(Double predictedRPM) {

    double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;

    // Track last good prediction
    if (predictedRPM != null) {
        lastPredictedRPM = predictedRPM;
    }

    // Toggle flywheel ON/OFF with gamepad2 Y
    if (gamepad2.yWasPressed()) {
        flywheelOn = !flywheelOn;

        if (flywheelOn) {
            double target = (predictedRPM != null) ? predictedRPM : lastPredictedRPM;
            flywheelTargetRPM = clamp(target, PRED_RPM_MIN, PRED_RPM_MAX);

            // >>> START KICK when turning ON
            startFlywheelKick();
        } else {
            // optional: clear kick state when turning off
            fwKickActive = false;
        }
    }

    // OFF behavior
    if (!flywheelOn) {
        fwKickActive = false;     // make sure kick is not “stuck”
        mFW.setPower(0.0);
        led.setPosition(LED_OFF_POS);
        telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
        telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
        return;
    }

    // ON behavior (kick then hold)
    boolean kicking = updateFlywheelKickHold(flywheelTargetRPM);

    boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;
    led.setPosition(atSpeed ? LED_GREEN_POS : LED_OFF_POS);

    telemetry.addData("Flywheel",
            "ON %s  Target=%.0f  Current=%.0f",
            kicking ? "(KICK)" : "(HOLD)",
            flywheelTargetRPM, currentRPM);
    telemetry.addData("AtSpeed", atSpeed ? "YES" : "NO");
}



//******************* Helper *****************

// Call when you turn flywheel ON (or whenever you want to re-kick)
private void startFlywheelKick() {
    fwKickActive = true;
    fwKickTimer.reset();

    mFW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    if (mFW.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    mFW.setPower(FW_KICK_POWER); // KICK
}

/**
 * Call every loop while flywheelOn is true.
 * Returns true if still in kick phase, false if in velocity-hold phase.
 */
private boolean updateFlywheelKickHold(double targetRPM) {
    double targetTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;

    // Still kicking?
    if (fwKickActive && fwKickTimer.milliseconds() < FW_KICK_MS) {
        // Keep max accel
        if (mFW.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        mFW.setPower(FW_KICK_POWER);
        return true;
    }

    // Kick done -> hold with velocity control
    fwKickActive = false;
    if (mFW.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    mFW.setVelocity(targetTicksPerSec);
    return false;
}

*/
