/*
 * Flywheel Spin-Up Comparison TeleOp (4500 RPM)
 *
 * Compares 3 methods and measures time-to-speed:
 *   1) setVelocity() using baseline (default/current) PIDF
 *   2) Kick-Hold: RUN_WITHOUT_ENCODER + setPower(1) for FW_KICK_MS, then setVelocity()
 *   3) setVelocity() with PIDF adjusted: P and/or F scaled +10/20/30% from baseline
 *
 * What it measures (you’ll see all in telemetry):
 *   - t90: time to reach 90% of target RPM
 *   - tTolStable: time to enter ±RPM_TOL and stay stable for STABLE_MS
 *
 * Controls:
 *   gamepad1 A  = run Method 1 (baseline setVelocity)
 *   gamepad1 B  = run Method 2 (kick-hold)
 *   gamepad1 X  = run Method 3 (PIDF scaled)
 *   gamepad1 START = run ALL 3 sequentially
 *
 *   gamepad1 dpad_up/down = select scale (1.10 / 1.20 / 1.30)
 *   gamepad1 left_bumper  = cycle tune mode: BOTH(P&F) -> P_ONLY -> F_ONLY
 *   gamepad1 Y = clear results
 *
 * Hardware config names:
 *   - Flywheel motor: "mFW"  (DcMotorEx)
 *   - LED servo: "led" (optional; code uses it if present)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Flywheel PIDF Compare (4500RPM)", group="Test")
public class FlywheelPidfCompareTeleOp extends LinearOpMode {

    // ---------- Hardware ----------
    private DcMotorEx mFW;
    private Servo led; // optional

    // ---------- Your constants ----------
    private static final double TICKS_PER_REV = 28.0;

    // LED positions (from you)
    private static final double LED_GREEN_POS = 0.5;
    private static final double LED_RED_POS   = 0.277;
    private static final double LED_OFF_POS   = 0.0;
    private static final double RPM_TOL       = 150.0;

    // ---------- Test target ----------
    private static final double TARGET_RPM = 4500.0;

    // ---------- Stability requirement ----------
    private static final long STABLE_MS = 120; // must stay within tolerance this long

    // ---------- Kick-Hold knobs ----------
    private static final long   FW_KICK_MS     = 300; // reduce if overshoot/hunting
    private static final double FW_KICK_POWER  = 1.0; // reduce if brownout/voltage dip

    // ---------- PIDF scaling choices ----------
    private final double[] scales = new double[] { 1.10, 1.20, 1.30 };
    private int scaleIdx = 0;

    private enum TuneMode { PF_BOTH, P_ONLY, F_ONLY }
    private TuneMode tuneMode = TuneMode.PF_BOTH;

    // ---------- Baseline coefficients (read at init) ----------
    private PIDFCoefficients basePidf = null;

    // ---------- Results ----------
    private String result1 = "not run";
    private String result2 = "not run";
    private String result3 = "not run";

    // ---------- State machine ----------
    private enum RunMode { IDLE, RUN_ONE, COOL_DOWN, RUN_ALL }
    private RunMode runMode = RunMode.IDLE;

    private enum Method { M1_BASELINE, M2_KICKHOLD, M3_SCALED }
    private Method currentMethod = Method.M1_BASELINE;

    private int allStep = 0; // 0->M1, 1->M2, 2->M3

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime methodTimer = new ElapsedTime();

    // Timing metrics for the current run
    private double t90 = -1;
    private double tTolStable = -1;
    private long tolEnterMs = -1; // when we first entered tolerance window

    // Button edge detection
    private boolean lastA=false, lastB=false, lastX=false, lastY=false, lastStart=false;
    private boolean lastUp=false, lastDown=false, lastLB=false;

    @Override
    public void runOpMode() {

        // --- map hardware ---
        mFW = hardwareMap.get(DcMotorEx.class, "mFW");

        // LED servo is optional: if not configured, code still runs
        try {
            led = hardwareMap.get(Servo.class, "led");
            led.setPosition(LED_OFF_POS);
        } catch (Exception e) {
            led = null;
        }

        // --- motor setup ---
        mFW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Read baseline PIDF for RUN_USING_ENCODER
        // (On most SDKs this corresponds to the velocity PIDF used by setVelocity().)
        basePidf = mFW.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready: Flywheel PIDF Compare (4500RPM)");
        telemetry.addLine("A=Method1  B=Method2  X=Method3  START=RunAll");
        telemetry.addLine("dpad_up/down choose scale; LB cycle tune mode; Y clear");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {

            // ---------- Read buttons (edge detect) ----------
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean start = gamepad1.start;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean lb = gamepad1.left_bumper;

            boolean aPressed = a && !lastA;
            boolean bPressed = b && !lastB;
            boolean xPressed = x && !lastX;
            boolean yPressed = y && !lastY;
            boolean startPressed = start && !lastStart;

            boolean upPressed = up && !lastUp;
            boolean downPressed = down && !lastDown;
            boolean lbPressed = lb && !lastLB;

            lastA=a; lastB=b; lastX=x; lastY=y; lastStart=start;
            lastUp=up; lastDown=down; lastLB=lb;

            // ---------- UI adjustments (only allowed when not running) ----------
            if (runMode == RunMode.IDLE) {
                if (upPressed)   scaleIdx = Math.min(scaleIdx + 1, scales.length - 1);
                if (downPressed) scaleIdx = Math.max(scaleIdx - 1, 0);

                if (lbPressed) {
                    switch (tuneMode) {
                        case PF_BOTH: tuneMode = TuneMode.P_ONLY; break;
                        case P_ONLY:  tuneMode = TuneMode.F_ONLY; break;
                        case F_ONLY:  tuneMode = TuneMode.PF_BOTH; break;
                    }
                }

                if (yPressed) {
                    result1 = "not run";
                    result2 = "not run";
                    result3 = "not run";
                }

                if (aPressed) beginSingle(Method.M1_BASELINE);
                if (bPressed) beginSingle(Method.M2_KICKHOLD);
                if (xPressed) beginSingle(Method.M3_SCALED);

                if (startPressed) beginAll();
            }

            // ---------- State machine run ----------
            switch (runMode) {
                case IDLE:
                    // keep motor off
                    safeStopFlywheel();
                    break;

                case RUN_ONE:
                    runCurrentMethod();
                    break;

                case COOL_DOWN:
                    // spin down for a moment between tests
                    safeStopFlywheel();
                    if (stateTimer.milliseconds() > 900) {
                        if (runMode == RunMode.COOL_DOWN && currentMethod == null) {
                            // should not happen
                            runMode = RunMode.IDLE;
                        } else {
                            if (allStep >= 0 && allStep <= 2 && runMode == RunMode.COOL_DOWN) {
                                // handled in RUN_ALL transition below
                            }
                        }
                    }
                    break;

                case RUN_ALL:
                    // RUN_ALL is orchestrated by chaining RUN_ONE and COOL_DOWN,
                    // so we won't do work directly here.
                    // (We still keep the switch for clarity.)
                    break;
            }

            // ---------- Telemetry ----------
            double rpm = getFlywheelRPM();
            boolean atSpeed = Math.abs(rpm - TARGET_RPM) <= RPM_TOL;

            if (led != null) {
                if (runMode == RunMode.IDLE) led.setPosition(LED_OFF_POS);
                else led.setPosition(atSpeed ? LED_GREEN_POS : LED_RED_POS);
            }

            telemetry.addData("TargetRPM", "%.0f", TARGET_RPM);
            telemetry.addData("CurrentRPM", "%.0f", rpm);
            telemetry.addData("Tol(±RPM)", "%.0f", RPM_TOL);

            telemetry.addData("Scale", "%.2fx", scales[scaleIdx]);
            telemetry.addData("TuneMode", tuneMode);

            telemetry.addData("Baseline PIDF", "P=%.3f I=%.3f D=%.3f F=%.3f",
                    basePidf.p, basePidf.i, basePidf.d, basePidf.f);

            telemetry.addData("State", runMode);
            telemetry.addData("Method", currentMethod);

            telemetry.addData("Result M1", result1);
            telemetry.addData("Result M2", result2);
            telemetry.addData("Result M3", result3);

            telemetry.update();
        }
    }

    // --------------------- Test Orchestration ---------------------

    private void beginSingle(Method m) {
        applyBaselinePidf(); // ensure baseline before each run (unless M3 changes it)
        currentMethod = m;
        resetMetrics();
        methodTimer.reset();
        stateTimer.reset();
        runMode = RunMode.RUN_ONE;
    }

    private void beginAll() {
        applyBaselinePidf();
        allStep = 0;
        runMode = RunMode.RUN_ALL;
        // Start first method immediately
        beginSingle(Method.M1_BASELINE);
        // But mark we are in ALL sequence
        runMode = RunMode.RUN_ONE; // actual work is RUN_ONE; sequence is tracked via allStep
    }

    private void finishRunAndRecord() {
        // Format results
        String msg = String.format("t90=%.0fms  tStable=%.0fms",
                t90 < 0 ? -1 : t90,
                tTolStable < 0 ? -1 : tTolStable);

        if (currentMethod == Method.M1_BASELINE) result1 = msg;
        if (currentMethod == Method.M2_KICKHOLD) result2 = msg;
        if (currentMethod == Method.M3_SCALED)   result3 = msg;

        // stop flywheel
        safeStopFlywheel();

        // If we are running ALL 3, chain next
        if (result1 != "not run" && result2 != "not run" && result3 != "not run") {
            runMode = RunMode.IDLE;
            return;
        }

        // Determine if we are in "run all" sequence by checking whether some results exist and others not.
        boolean runningAll = (result1 != "not run" && (result2.equals("not run") || result3.equals("not run")))
                || (result1.equals("not run") && result2.equals("not run") && result3.equals("not run")); // start case

        // If user launched single method, just go idle
        // We detect "single" by: if only this method was requested (no sequencing).
        // Simpler: if START was used, results will be filled sequentially; otherwise user will press buttons.
        // We'll implement a robust sequence based on which results are still "not run".

        // If any of the other results are still "not run", continue sequence in fixed order.
        if (!runningAll) {
            runMode = RunMode.IDLE;
            return;
        }

        // Cool down then start next missing method in order M1->M2->M3
        runMode = RunMode.COOL_DOWN;
        stateTimer.reset();

        // After cooldown, begin next
        // We'll do the transition by checking in runCurrentMethod() after cooldown time.
    }

    // --------------------- Core Method Runner ---------------------

    private void runCurrentMethod() {
        // Transition from COOL_DOWN to next method if needed
        if (runMode == RunMode.COOL_DOWN) {
            if (stateTimer.milliseconds() > 900) {
                if (result1.equals("not run")) beginSingle(Method.M1_BASELINE);
                else if (result2.equals("not run")) beginSingle(Method.M2_KICKHOLD);
                else if (result3.equals("not run")) beginSingle(Method.M3_SCALED);
                else runMode = RunMode.IDLE;
            }
            return;
        }

        // Apply commanded behavior each loop
        switch (currentMethod) {
            case M1_BASELINE:
                applyBaselinePidf();
                holdVelocitySetVelocity(TARGET_RPM);
                break;

            case M2_KICKHOLD:
                applyBaselinePidf();
                kickHold(TARGET_RPM);
                break;

            case M3_SCALED:
                applyScaledPidf(scales[scaleIdx], tuneMode);
                holdVelocitySetVelocity(TARGET_RPM);
                break;
        }

        // Update metrics (time to 90% and time to stable tolerance)
        updateMetrics();

        // If we reached stable tolerance, finish
        if (tTolStable >= 0) {
            finishRunAndRecord();

            // If there are remaining methods to run (RUN ALL), go to cooldown
            if (result1.equals("not run") || result2.equals("not run") || result3.equals("not run")) {
                runMode = RunMode.COOL_DOWN;
                stateTimer.reset();
            } else {
                runMode = RunMode.IDLE;
            }
        }
    }

    // --------------------- Behaviors ---------------------

    // Method 1/3: just use setVelocity in RUN_USING_ENCODER
    private void holdVelocitySetVelocity(double targetRPM) {
        double targetTPS = targetRPM * TICKS_PER_REV / 60.0;
        if (mFW.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        mFW.setVelocity(targetTPS);
    }

    // Method 2: kick then hold
    private void kickHold(double targetRPM) {
        double targetTPS = targetRPM * TICKS_PER_REV / 60.0;

        if (methodTimer.milliseconds() < FW_KICK_MS) {
            if (mFW.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            mFW.setPower(FW_KICK_POWER);
        } else {
            if (mFW.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            mFW.setVelocity(targetTPS);
        }
    }

    // --------------------- PIDF helpers ---------------------

    private void applyBaselinePidf() {
        // Re-apply baseline to ensure we're really using the default/current values
        if (basePidf != null) {
            mFW.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, basePidf);
        }
    }

    private void applyScaledPidf(double scale, TuneMode mode) {
        if (basePidf == null) return;

        PIDFCoefficients c = new PIDFCoefficients(basePidf.p, basePidf.i, basePidf.d, basePidf.f);

        switch (mode) {
            case PF_BOTH:
                c.p = basePidf.p * scale;
                c.f = basePidf.f * scale;
                break;
            case P_ONLY:
                c.p = basePidf.p * scale;
                break;
            case F_ONLY:
                c.f = basePidf.f * scale;
                break;
        }

        mFW.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

    // --------------------- Metrics ---------------------

    private void resetMetrics() {
        t90 = -1;
        tTolStable = -1;
        tolEnterMs = -1;
    }

    private void updateMetrics() {
        double rpm = getFlywheelRPM();
        double ms = methodTimer.milliseconds();

        // time to 90%
        if (t90 < 0 && rpm >= 0.90 * TARGET_RPM) {
            t90 = ms;
        }

        // stable tolerance timing
        boolean inTol = Math.abs(rpm - TARGET_RPM) <= RPM_TOL;

        if (tTolStable < 0) {
            if (inTol) {
                if (tolEnterMs < 0) {
                    tolEnterMs = (long) ms;
                } else {
                    long dur = (long) ms - tolEnterMs;
                    if (dur >= STABLE_MS) {
                        tTolStable = ms;
                    }
                }
            } else {
                tolEnterMs = -1; // reset if we leave tolerance window
            }
        }
    }

    // --------------------- Utilities ---------------------

    private double getFlywheelRPM() {
        // getVelocity() returns ticks/sec
        return Math.abs(mFW.getVelocity()) * 60.0 / TICKS_PER_REV;
    }

    private void safeStopFlywheel() {
        // Avoid mode thrash; just stop
        mFW.setPower(0.0);
        // Keep LED off handled in loop
    }
}
