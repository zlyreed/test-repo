package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * TeleOpAimPoseRPM_speedFunc
 * - Limelight trig-distance -> predicted RPM
 * - Flywheel control with LED at-speed indicator
 * - Manual intake/RW1/RW2 toggles + RW2 manual pulse (LB) + unjam (hold X)
 * - Auto-Shoot (hold gamepad2 RIGHT TRIGGER): RPM-dip detection feeds one ball per shot
 * - Auto-Load (hold gamepad2 LEFT TRIGGER): ratchet stack balls while driving
 *    * When LT released: AUTO-HOLD keeps intake holding + RW2 gated (even if intakeOn was false)
 *
 * Priority:
 * 1) Unjam (hold X) -> manual unjam wins
 * 2) Auto-Shoot (RT)
 * 3) Auto-Load / Auto-Hold (LT or hold mode after release)
 * 4) Manual controls
 */
@TeleOp(name = "TeleOpAimPoseRPM_speedFunc", group = "Robot")
public class TeleOpAimPoseRPM_speedFunc extends LinearOpMode {

    // =========================
    // Hardware
    // =========================
    private DcMotor mFL, mFR, mBL, mBR;
    private DcMotorEx mFW;
    private CRServo sI, sRW1, sRW2;
    private IMU imu;
    private Limelight3A limelight;
    private Servo led;

    // =========================
    // Constants (tune)
    // =========================
    private static final double TICKS_PER_REV = 28;       // change to your motor spec!
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    // Aim assist tuning
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_TURN = 0.35;

    // -------- Trig distance constants (inches + degrees) --------
    private static final double GOAL_TAG_CENTER_HEIGHT_IN = 29.49; // measured in field
    private static final double LL_LENS_HEIGHT_IN = 14.5;          // measured on robot
    private static final double LL_MOUNT_ANGLE_DEG = 0;            // camera pitch up from horizontal
    private static final double LL_LATERAL_OFFSET_IN = 4.5;        // camera lateral offset (inches)

    // -------- Flywheel speed prediction model (distanceIn inches -> rpm) --------
    // Linear regression: y=16.57766x+2602.65546
    private static final double RPM_SLOPE = 16.57766;
    private static final double RPM_INTERCEPT = 2602.65546;

    private static final double PRED_RPM_MIN = 1500;
    private static final double PRED_RPM_MAX = 5800;

    // Manual fallback RPMs if goal not seen
    private static final double MANUAL_RPM_UP = 3800;    // near
    private static final double MANUAL_RPM_DOWN = 5000;  // far

    // ---------------- LED light -------------------
    private static final double LED_GREEN_POS = 0.5;   // Green
    private static final double LED_RED_POS = 0.277;   // Red (optional)
    private static final double LED_OFF_POS = 0.0;     // off (try 1 if 0 doesn't work)
    private static final double LED_RPM_TOL = 150;     // speed tolerance

    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID;
    private boolean flywheelOn = false;

    private double flywheelTargetRPM = MANUAL_RPM_UP;
    private double lastPredictedRPM = MANUAL_RPM_UP;
    private double manualFallbackRPM = MANUAL_RPM_UP;

    // Manual toggles (only apply when neither auto is controlling servos)
    private boolean intakeOn = false;
    private boolean rw1On = false;
    private boolean rw2On = false;

    // =========================
    // Manual RW2 pulse (LB)
    // =========================
    private static final long RW2_PULSE_MS = 150;       // tune
    private static final double RW2_PULSE_PWR = 1.0;

    private boolean rw2PulseActive = false;
    private long rw2PulseStartMs = 0;

    // =========================
    // "Hold / gate" tuning (used by manual gating + auto modes)
    // =========================
    private static final double INTAKE_HOLD_PWR = 0.18;      // keeps balls from falling out
    private static final double RW2_GATE_HOLD_PWR = -0.08;   // tiny reverse gate (set 0 if jams)

    // =========================
    // AUTO-SHOOT (hold RIGHT TRIGGER) + shot detection via RPM dip
    // =========================
    private static final double AUTO_SHOOT_TRIGGER_THRESH = 0.6;

    private static final double SHOT_DIP_RPM     = 250;   // tune: 200–400
    private static final long   SHOT_DEBOUNCE_MS = 60;
    private static final long   MAX_FEED_TIME_MS = 500;

    private static final long GATE_HOLD_MS   = 180;
    private static final long RESTAGE_MS     = 320;
    private static final long RECOVER_MIN_MS = 150;

    private static final double INTAKE_RESTAGE_PWR = 0.60;
    private static final double RW1_RESTAGE_PWR    = 1.00;

    private enum AutoShootState { IDLE, WAIT_READY, FEED_UNTIL_SHOT, GATE_HOLD, RESTAGE, WAIT_RECOVER }
    private AutoShootState autoShootState = AutoShootState.IDLE;
    private long autoShootStateStartMs = 0;

    private long dipStartMs = -1;
    private boolean shotDetectedThisCycle = false;

    // =========================
    // AUTO-LOAD (hold LEFT TRIGGER) + AUTO-HOLD after release
    // =========================
    private static final double AUTO_LOAD_TRIGGER_THRESH = 0.6;

    private static final long LOAD_LIFT_MS   = 350; // RW1 ON
    private static final long LOAD_SETTLE_MS = 180; // RW1 OFF

    private static final double LOAD_INTAKE_PWR = 1.0;
    private static final double LOAD_RW1_PWR    = 1.0;

    private enum AutoLoadState { IDLE, LIFT, SETTLE, HOLD }
    private AutoLoadState autoLoadState = AutoLoadState.IDLE;
    private long autoLoadStateStartMs = 0;

    /**
     * AUTO-HOLD latch:
     * - If driver has used Auto-Load recently (LT held) and then releases LT,
     *   we keep holding balls (INTAKE_HOLD + RW2 gate) until:
     *   - Auto-Shoot takes over, OR
     *   - Driver enters Auto-Load again, OR
     *   - Driver holds X (unjam), OR
     *   - (optional) you add a cancel button later
     *
     * Per your choice A: RB cannot stop intake during Auto-Hold.
     */
    private boolean autoHoldLatched = false;

    // For field-centric drive
    private double initYawDeg = 0;

    @Override
    public void runOpMode() {

        initHardware();
        initYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Ready.");
        telemetry.addLine("AimAssist: hold gamepad1 LEFT TRIGGER.");
        telemetry.addLine("Goal: gamepad1 X=Blue(20), B=Red(24).");
        telemetry.addLine("Auto-Shoot: hold gamepad2 RIGHT TRIGGER.");
        telemetry.addLine("Auto-Load: hold gamepad2 LEFT TRIGGER. Release -> Auto-HOLD keeps balls.");
        telemetry.addLine("Manual RW2 pulse: gamepad2 LB. Unjam: hold gamepad2 X.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Choose goal tag
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Drive + aim assist
            driveFieldCentricWithOptionalAimAssist();

            // 3) Limelight predicted RPM
            Double predictedRPM = updateLimelightPoseAndDistanceTelemetry(goalTagId);

            // 4) Flywheel control
            applyFlywheelControl(predictedRPM);

            // 5) Decide who owns intake/RW1/RW2 this loop
            boolean unjamHeld = gamepad2.x;

            boolean autoShootHeld = (gamepad2.right_trigger > AUTO_SHOOT_TRIGGER_THRESH);
            boolean autoLoadHeld  = (gamepad2.left_trigger  > AUTO_LOAD_TRIGGER_THRESH);

            boolean autoOverride = false;

            // Unjam always wins: go manual unjam behavior
            if (unjamHeld) {
                // Cancel hold latches while unjamming (safe)
                autoHoldLatched = false;
                autoShootState = AutoShootState.IDLE;
                autoLoadState  = AutoLoadState.IDLE;

                // Manual unjam action (simple)
                sI.setPower(0.0);
                sRW1.setPower(-0.25);
                sRW2.setPower(-0.25);

                telemetry.addData("Mode", "UNJAM (X held)");
                autoOverride = true;
            }

            // Auto-Shoot has priority over Auto-Load / Auto-Hold
            if (!autoOverride) {
                autoOverride = updateAutoShootStateMachine(autoShootHeld);
                if (autoOverride) {
                    autoHoldLatched = false; // shooting mode owns everything; stop hold
                    telemetry.addData("Mode", "AUTO-SHOOT");
                }
            }

            // Auto-Load (LT held) OR Auto-Hold latched (after LT release)
            if (!autoOverride) {
                boolean loadOverride = updateAutoLoadStateMachine(autoLoadHeld);
                if (loadOverride) {
                    autoOverride = true;
                    telemetry.addData("Mode", autoLoadHeld ? "AUTO-LOAD" : "AUTO-HOLD");
                }
            }

            // Manual controls only if no auto is controlling servos
            if (!autoOverride) {
                updateIntakeAndRampWheelControls();
                telemetry.addData("Mode", "MANUAL");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    // =========================================================
    // HARDWARE INIT
    // =========================================================
    private void initHardware() {
        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sI = hardwareMap.get(CRServo.class, "sI");
        sRW1 = hardwareMap.get(CRServo.class, "sRW1");
        sRW2 = hardwareMap.get(CRServo.class, "sRW2");

        imu = hardwareMap.get(IMU.class, "imu");

        // Directions: keep your working settings
        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        sI.setDirection(CRServo.Direction.REVERSE);
        sRW1.setDirection(CRServo.Direction.REVERSE);
        sRW2.setDirection(CRServo.Direction.REVERSE);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(7);
        limelight.start();

        // LED (servo-like controller)
        led = hardwareMap.get(Servo.class, "led");
    }

    // =========================================================
    // DRIVE + AIM ASSIST
    // =========================================================
    private void driveFieldCentricWithOptionalAimAssist() {

        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double yawDeltaDeg = yawDeg - initYawDeg;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turnManual = gamepad1.right_stick_x;

        // rotate (x,y) by -yawDelta
        double cosA = Math.cos(Math.toRadians(-yawDeltaDeg));
        double sinA = Math.sin(Math.toRadians(-yawDeltaDeg));
        double xRot = (x * cosA) - (y * sinA);
        double yRot = (x * sinA) + (y * cosA);

        double turnAssist = 0.0;
        boolean aimAssistEnabled = gamepad1.left_trigger > 0.2;

        LLResult result = limelight.getLatestResult();

        if (aimAssistEnabled) {
            Double txDeg = getTxToGoalTag(goalTagId, result);

            // optional lateral-offset setpoint if ty & trig distance available
            Double tyDeg = getTyToGoalTag(goalTagId, result);
            Double trigDistIn = (tyDeg != null) ? trigDistanceToGoalInchesFromTy(tyDeg) : null;

            if (txDeg != null && trigDistIn != null) {
                double txSetpointDeg = Math.toDegrees(Math.atan2(LL_LATERAL_OFFSET_IN, trigDistIn));
                double txErrorDeg = txDeg - txSetpointDeg;
                turnAssist = clamp(txErrorDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);

                telemetry.addData("AimAssist", "ON tx=%.1f set=%.1f err=%.1f turn=%.2f",
                        txDeg, txSetpointDeg, txErrorDeg, turnAssist);
            } else if (txDeg != null) {
                turnAssist = clamp(txDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
                telemetry.addData("AimAssist", "ON tx=%.1f turn=%.2f (no trig dist)", txDeg, turnAssist);
            } else {
                telemetry.addData("AimAssist", "ON (no goal tag)");
            }
        } else {
            telemetry.addData("AimAssist", "OFF");
        }

        double turn = turnManual + turnAssist;

        // mecanum mixing
        double pFL = yRot + xRot + turn;
        double pFR = yRot - xRot - turn;
        double pBL = yRot - xRot + turn;
        double pBR = yRot + xRot - turn;

        // normalize
        double max = Math.max(Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR)));
        if (max > 1.0) { pFL /= max; pFR /= max; pBL /= max; pBR /= max; }

        mFL.setPower(pFL);
        mFR.setPower(pFR);
        mBL.setPower(pBL);
        mBR.setPower(pBR);

        telemetry.addData("Goal Tag", goalTagId);
        telemetry.addData("IMU Yaw", "%.1f°", yawDeg);
    }

    // =========================================================
    // LIMELIGHT TRIG DIST + PREDICTED RPM (or manual fallback)
    // =========================================================
    private Double updateLimelightPoseAndDistanceTelemetry(int goalTagId) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("LL", "No valid result");
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        telemetry.addData("LL Pipeline", "Index=%d Type=%s",
                result.getPipelineIndex(), result.getPipelineType());

        Double tyDeg = getTyToGoalTag(goalTagId, result);
        Double txDeg = getTxToGoalTag(goalTagId, result);

        if (tyDeg == null) {
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "n/a (goal tag %d not in view)", goalTagId);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);
        if (trigDistIn == null) {
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "invalid (ty=%.2f°)", tyDeg);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        double predictedRPM = RPM_SLOPE * trigDistIn + RPM_INTERCEPT;
        predictedRPM = clamp(predictedRPM, PRED_RPM_MIN, PRED_RPM_MAX);

        telemetry.addData("TrigDist", "%.1f in (Tx=%.1f Ty=%.1f) to Goal %d",
                trigDistIn, (txDeg != null ? txDeg : 999.0), tyDeg, goalTagId);
        telemetry.addData("PredRPM", "%.0f RPM", predictedRPM);
        telemetry.addData("ManualRPM", "%.0f (use only if goal not seen)", manualFallbackRPM);

        return predictedRPM;
    }

    // =========================================================
    // FLYWHEEL CONTROL (toggle gamepad2 Y)
    // =========================================================
    private void applyFlywheelControl(Double predictedRPM) {

        double currentRPM = getFlywheelRPM();

        if (predictedRPM != null) {
            lastPredictedRPM = predictedRPM;
        }

        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                double target = (predictedRPM != null) ? predictedRPM : lastPredictedRPM;
                flywheelTargetRPM = clamp(target, PRED_RPM_MIN, PRED_RPM_MAX);
            }
        }

        if (!flywheelOn) {
            mFW.setPower(0.0);
            led.setPosition(LED_OFF_POS);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
            return;
        }

        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);

        boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;
        led.setPosition(atSpeed ? LED_GREEN_POS : LED_OFF_POS);

        telemetry.addData("Flywheel", "ON  Target=%.0f  Current=%.0f", flywheelTargetRPM, currentRPM);
        telemetry.addData("AtSpeed", atSpeed ? "YES" : "NO");
    }

    // =========================================================
    // MANUAL controls (only used when no auto owns servos)
    // =========================================================
    private void updateIntakeAndRampWheelControls() {

        long now = System.currentTimeMillis();

        // Toggles
        if (gamepad2.rightBumperWasPressed()) intakeOn = !intakeOn;
        if (gamepad1.rightBumperWasPressed()) rw1On = !rw1On;
        if (gamepad2.aWasPressed())           rw2On = !rw2On;

        // Manual RW2 pulse
        if (gamepad2.leftBumperWasPressed()) {
            rw2PulseActive = true;
            rw2PulseStartMs = now;
        }
        if (rw2PulseActive && (now - rw2PulseStartMs >= RW2_PULSE_MS)) {
            rw2PulseActive = false;
        }

        boolean ready = flywheelReady();

        // Base powers
        double intakePower = intakeOn ? 1.0 : 0.0;
        double rw1Power    = rw1On ? 1.0 : 0.0;
        double rw2Power    = rw2On ? 1.0 : 0.0;

        // If flywheel ON but NOT ready: keep intake at hold, pause RW1, gate RW2
        if (flywheelOn && !ready) {
            intakePower = intakeOn ? INTAKE_HOLD_PWR : 0.0;
            rw1Power = 0.0;
            rw2Power = RW2_GATE_HOLD_PWR;
        }

        // Manual pulse overrides RW2 and pauses RW1
        if (rw2PulseActive) {
            rw2Power = RW2_PULSE_PWR;
            rw1Power = 0.0;
            if (intakeOn) intakePower = Math.min(intakePower, INTAKE_HOLD_PWR);
        }

        sI.setPower(intakePower);
        sRW1.setPower(rw1Power);
        sRW2.setPower(rw2Power);

        telemetry.addData("Manual", "I=%.2f RW1=%.2f RW2=%.2f", intakePower, rw1Power, rw2Power);
        telemetry.addData("RW2 Pulse", rw2PulseActive ? "ACTIVE" : "OFF");
    }

    // =========================================================
    // AUTO-SHOOT: hold RT
    // Returns true if auto-shoot owns servos this loop.
    // =========================================================
    private boolean updateAutoShootStateMachine(boolean autoShootHeld) {

        // If not held and idle, don't own
        if (!autoShootHeld && autoShootState == AutoShootState.IDLE) return false;

        long now = System.currentTimeMillis();
        long dt  = now - autoShootStateStartMs;

        double rpmNow = getFlywheelRPM();
        boolean ready = flywheelReady();

        // Release RT -> stop shooting and yield
        if (!autoShootHeld && autoShootState != AutoShootState.IDLE) {
            // Safe: keep balls held (intake hold + RW2 gate)
            sI.setPower(INTAKE_HOLD_PWR);
            sRW1.setPower(0.0);
            sRW2.setPower(RW2_GATE_HOLD_PWR);

            dipStartMs = -1;
            shotDetectedThisCycle = false;
            enterAutoShootState(AutoShootState.IDLE);
            return false;
        }

        // Auto-shoot is active; it owns servos
        switch (autoShootState) {

            case IDLE:
                sI.setPower(INTAKE_HOLD_PWR);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                dipStartMs = -1;
                shotDetectedThisCycle = false;
                enterAutoShootState(AutoShootState.WAIT_READY);
                break;

            case WAIT_READY:
                sI.setPower(INTAKE_HOLD_PWR);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (ready) {
                    dipStartMs = -1;
                    shotDetectedThisCycle = false;
                    sRW2.setPower(RW2_PULSE_PWR);
                    enterAutoShootState(AutoShootState.FEED_UNTIL_SHOT);
                }
                break;

            case FEED_UNTIL_SHOT:
                sI.setPower(INTAKE_HOLD_PWR);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_PULSE_PWR);

                if (!shotDetectedThisCycle && detectShotDip(rpmNow)) {
                    shotDetectedThisCycle = true;
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoShootState(AutoShootState.GATE_HOLD);
                }

                if (dt >= MAX_FEED_TIME_MS) {
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoShootState(AutoShootState.GATE_HOLD);
                }
                break;

            case GATE_HOLD:
                sI.setPower(INTAKE_HOLD_PWR);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= GATE_HOLD_MS) {
                    enterAutoShootState(AutoShootState.RESTAGE);
                }
                break;

            case RESTAGE:
                // Push next ball up (but keep RW2 gated)
                sI.setPower(INTAKE_RESTAGE_PWR);
                sRW1.setPower(RW1_RESTAGE_PWR);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= RESTAGE_MS) {
                    sI.setPower(INTAKE_HOLD_PWR);
                    sRW1.setPower(0.0);
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoShootState(AutoShootState.WAIT_RECOVER);
                }
                break;

            case WAIT_RECOVER:
                sI.setPower(INTAKE_HOLD_PWR);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= RECOVER_MIN_MS && ready) {
                    dipStartMs = -1;
                    shotDetectedThisCycle = false;
                    sRW2.setPower(RW2_PULSE_PWR);
                    enterAutoShootState(AutoShootState.FEED_UNTIL_SHOT);
                }
                break;
        }

        telemetry.addData("AutoShoot", "%s", autoShootState);
        telemetry.addData("RPM", "%.0f", rpmNow);
        telemetry.addData("DipThresh", "%.0f", (flywheelTargetRPM - SHOT_DIP_RPM));
        telemetry.addData("FlywheelReady", ready);

        return true;
    }

    private void enterAutoShootState(AutoShootState s) {
        autoShootState = s;
        autoShootStateStartMs = System.currentTimeMillis();
    }

    // =========================================================
    // AUTO-LOAD + AUTO-HOLD: hold LT to load; release to hold
    // Returns true if auto-load/hold owns servos this loop.
    // =========================================================
    private boolean updateAutoLoadStateMachine(boolean autoLoadHeld) {

        // If neither held nor latched, don't own
        if (!autoLoadHeld && !autoHoldLatched && autoLoadState == AutoLoadState.IDLE) return false;

        long now = System.currentTimeMillis();
        long dt  = now - autoLoadStateStartMs;

        // If held: we are actively loading; latch hold for after release
        if (autoLoadHeld) {
            autoHoldLatched = true;

            switch (autoLoadState) {
                case IDLE:
                case HOLD:
                    enterAutoLoadState(AutoLoadState.LIFT);
                    break;

                case LIFT:
                    sI.setPower(LOAD_INTAKE_PWR);
                    sRW1.setPower(LOAD_RW1_PWR);
                    sRW2.setPower(RW2_GATE_HOLD_PWR);

                    if (dt >= LOAD_LIFT_MS) {
                        enterAutoLoadState(AutoLoadState.SETTLE);
                    }
                    break;

                case SETTLE:
                    sI.setPower(LOAD_INTAKE_PWR);
                    sRW1.setPower(0.0);
                    sRW2.setPower(RW2_GATE_HOLD_PWR);

                    if (dt >= LOAD_SETTLE_MS) {
                        enterAutoLoadState(AutoLoadState.LIFT);
                    }
                    break;
            }

            telemetry.addData("AutoLoad", "%s", autoLoadState);
            return true;
        }

        // Not held:
        // If hold latched, we do AUTO-HOLD (your choice A):
        // Always hold intake + gate RW2, regardless of intakeOn toggle.
        if (autoHoldLatched) {
            sI.setPower(INTAKE_HOLD_PWR);
            sRW1.setPower(0.0);
            sRW2.setPower(RW2_GATE_HOLD_PWR);

            enterAutoLoadState(AutoLoadState.HOLD);
            telemetry.addData("AutoLoad", "HOLD (latched)");
            return true;
        }

        // Shouldn't reach here often, but safe fallback
        return false;
    }

    private void enterAutoLoadState(AutoLoadState s) {
        autoLoadState = s;
        autoLoadStateStartMs = System.currentTimeMillis();
    }

    // =========================================================
    // Helpers
    // =========================================================
    private double getFlywheelRPM() {
        return mFW.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private boolean flywheelReady() {
        if (!flywheelOn) return false;
        double rpm = getFlywheelRPM();
        return Math.abs(rpm - flywheelTargetRPM) <= LED_RPM_TOL;
    }

    private boolean detectShotDip(double rpmNow) {
        double dipThreshold = flywheelTargetRPM - SHOT_DIP_RPM;

        if (rpmNow < dipThreshold) {
            if (dipStartMs < 0) dipStartMs = System.currentTimeMillis();
            long dt = System.currentTimeMillis() - dipStartMs;
            if (dt >= SHOT_DEBOUNCE_MS) return true;
        } else {
            dipStartMs = -1;
        }
        return false;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private Double getTxToGoalTag(int desiredId, LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == desiredId) {
                return f.getTargetXDegrees();
            }
        }
        return null;
    }

    private Double getTyToGoalTag(int desiredId, LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == desiredId) {
                return f.getTargetYDegrees();
            }
        }
        return null;
    }

    private Double trigDistanceToGoalInchesFromTy(double tyDeg) {
        double totalAngleDeg = LL_MOUNT_ANGLE_DEG + tyDeg;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        double heightDiffIn = GOAL_TAG_CENTER_HEIGHT_IN - LL_LENS_HEIGHT_IN;

        double tanVal = Math.tan(totalAngleRad);
        if (Math.abs(tanVal) < 1e-6) return null;

        double dIn = heightDiffIn / tanVal;
        if (dIn <= 0) return null;
        return dIn;
    }
}
