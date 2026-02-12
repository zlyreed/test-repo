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

// TeleOpAimPoseRPM_speedFunc_autoShoot
// - Limelight trig-distance -> predicted RPM
// - Flywheel control with LED at-speed indicator
// - Manual intake/RW1/RW2 toggles + RW2 manual pulse (LB) + unjam (hold X)
// - NEW: Auto-shoot state machine (hold gamepad2 RIGHT TRIGGER)
//        Uses flywheel RPM dip detection to feed until a shot is detected,
//        then gate-hold, restage, recover, and repeat.
//        Auto & manual never fight over servos.

@TeleOp(name = "TeleOpAimPoseRPM_speedFunc_autoShoot", group = "Robot")
public class TeleOpAimPoseRPM_speedFunc_autoShoot extends LinearOpMode {

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
    private static final double GOAL_TAG_CENTER_HEIGHT_IN = 29.49; // measured in the field
    private static final double LL_LENS_HEIGHT_IN = 14.5;          // measured on robot
    private static final double LL_MOUNT_ANGLE_DEG = 0;            // camera pitch up from horizontal
    private static final double LL_LATERAL_OFFSET_IN = 4.5;        // camera from robot center (inches)

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

    // Flywheel targets
    private double flywheelTargetRPM = MANUAL_RPM_UP;
    private double lastPredictedRPM = MANUAL_RPM_UP;
    private double manualFallbackRPM = MANUAL_RPM_UP;

    // Intake / RW toggles
    private boolean intakeOn = false;
    private boolean rw1On = false;
    private boolean rw2On = false;

    // =========================
    // RW2 manual pulse (LB) settings
    // =========================
    private static final long RW2_PULSE_MS = 150;       // tune
    private static final double RW2_PULSE_PWR = 1.0;

    private boolean rw2PulseActive = false;
    private long rw2PulseStartMs = 0;

    // =========================
    // Manual-mode “hold / gate” tuning (prevents early touch while flywheel not ready)
    // =========================
    private static final double INTAKE_HOLD_PWR = 0.8;      // keeps balls from falling out (tune 0.10–0.30)
    private static final double RW2_GATE_HOLD_PWR = -0.05;   // tiny reverse gate (set 0.0 if jams)

    // =========================
    // AUTO-SHOOT (hold RIGHT TRIGGER) + shot detection via RPM dip
    // =========================
    private static final double AUTO_SHOOT_TRIGGER_THRESH = 0.6; // hold gamepad2.right_trigger > this

    // Shot detection
    private static final double SHOT_DIP_RPM     = 250;   // tune: 200–400
    private static final long   SHOT_DEBOUNCE_MS = 60;    // dip must persist this long
    private static final long   MAX_FEED_TIME_MS = 500;   // safety timeout

    // Auto-shoot timing
    private static final long GATE_HOLD_MS   = 100;
    private static final long RESTAGE_MS     = 250;
    private static final long RECOVER_MIN_MS = 80;

    // Auto restage powers
    private static final double INTAKE_RESTAGE_PWR = 1.0;
    private static final double RW1_RESTAGE_PWR    = 0.8;

    private enum AutoShootState { IDLE, WAIT_READY, FEED_UNTIL_SHOT, GATE_HOLD, RESTAGE, WAIT_RECOVER }
    private AutoShootState autoState = AutoShootState.IDLE;
    private long autoStateStartMs = 0;

    // Dip detector state
    private long dipStartMs = -1;
    private boolean shotDetectedThisCycle = false;

    // For field-centric drive
    private double initYawDeg = 0;

    @Override
    public void runOpMode() {

        initHardware();

        initYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Ready. Drive to position.");
        telemetry.addLine("Aim: hold gamepad1 LEFT TRIGGER for aim assist to goal tag.");
        telemetry.addLine("Goal tag: gamepad1 X=Blue(20), B=Red(24)");
        telemetry.addLine("Auto-shoot: hold gamepad2 RIGHT TRIGGER (uses RPM dip detection).");
        telemetry.addLine("Manual RW2 pulse: gamepad2 LEFT BUMPER. Unjam: hold gamepad2 X.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Pick which goal tag you are aiming at
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Manual drive + optional aim assist overlay
            driveFieldCentricWithOptionalAimAssist();

            // 3) Limelight trig distance + predicted RPM (or manual fallback if no tag)
            Double predictedRPM = updateLimelightPoseAndDistanceTelemetry(goalTagId);

            // 4) Flywheel control (toggle Y); LED green when within tolerance
            applyFlywheelControl(predictedRPM);

            // 5) Auto-shoot state machine OR manual intake/RW control
            boolean autoShootHeld = (gamepad2.right_trigger > AUTO_SHOOT_TRIGGER_THRESH);

            boolean autoOverride = updateAutoShootStateMachine(autoShootHeld);

            if (!autoOverride) {
                updateIntakeAndRampWheelControls();
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
        limelight.pipelineSwitch(7);  // AprilTag pipeline
        limelight.start();

        // LED (servo-like controller)
        led = hardwareMap.get(Servo.class, "led");
    }

    // =========================================================
    // DRIVE + AIM ASSIST
    // =========================================================
    private void driveFieldCentricWithOptionalAimAssist() {

        // Field-centric drive from IMU
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double yawDeltaDeg = yawDeg - initYawDeg;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turnManual = gamepad1.right_stick_x;

        // Rotate (x,y) by -yawDelta
        double cosA = Math.cos(Math.toRadians(-yawDeltaDeg));
        double sinA = Math.sin(Math.toRadians(-yawDeltaDeg));
        double xRot = (x * cosA) - (y * sinA);
        double yRot = (x * sinA) + (y * cosA);

        // Optional aim assist: hold left trigger
        double turnAssist = 0.0;
        boolean aimAssistEnabled = gamepad1.left_trigger > 0.2;

        LLResult result = limelight.getLatestResult();

        if (aimAssistEnabled) {
            Double txDeg = getTxToGoalTag(goalTagId, result);

            // If additional adjustment is used: need ty + trig distance
            Double tyDeg = getTyToGoalTag(goalTagId, result);
            Double trigDistIn = (tyDeg != null) ? trigDistanceToGoalInchesFromTy(tyDeg) : null;

            if (txDeg != null && trigDistIn != null) {
                // txSetpoint = atan(offset / distance)
                double txSetpointDeg = Math.toDegrees(Math.atan2(LL_LATERAL_OFFSET_IN, trigDistIn));
                double txErrorDeg = txDeg - txSetpointDeg;
                turnAssist = clamp(txErrorDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);

                telemetry.addData("AimAssist", "ON tx=%.1f° set=%.1f° err=%.1f° turn=%.2f",
                        txDeg, txSetpointDeg, txErrorDeg, turnAssist);
            } else if (txDeg != null) {
                // Fall back to simple tx=0 targeting if distance not available
                turnAssist = clamp(txDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
                telemetry.addData("AimAssist", "ON tx=%.1f° turn=%.2f (no trig dist)", txDeg, turnAssist);
            } else {
                telemetry.addData("AimAssist", "ON (no goal tag in view)");
            }
        } else {
            telemetry.addData("AimAssist", "OFF");
        }

        double turn = turnManual + turnAssist;

        // Mecanum mixing
        double pFL = yRot + xRot + turn;
        double pFR = yRot - xRot - turn;
        double pBL = yRot - xRot + turn;
        double pBR = yRot + xRot - turn;

        // Normalize
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

            // Manual select (held dpad, simple & reliable)
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

        // Predicted RPM from your linear model
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
            }
        }

        // OFF behavior
        if (!flywheelOn) {
            mFW.setPower(0.0);
            led.setPosition(LED_OFF_POS);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
            return;
        }

        // ON behavior
        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);

        boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;
        led.setPosition(atSpeed ? LED_GREEN_POS : LED_OFF_POS);

        telemetry.addData("Flywheel", "ON  Target=%.0f  Current=%.0f", flywheelTargetRPM, currentRPM);
        telemetry.addData("AtSpeed", atSpeed ? "YES" : "NO");
    }

    // =========================================================
    // MANUAL Intake + RW1 + RW2 controls
    // - Intake toggle: gamepad2 RB
    // - RW1 toggle:    gamepad1 RB
    // - RW2 toggle:    gamepad2 A
    // - RW2 pulse:     gamepad2 LB
    // - Unjam:         hold gamepad2 X
    // Notes:
    // - If flywheelOn && NOT ready: keep intake at hold power, pause RW1, gate RW2
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

        // Unjam
        boolean unjamHeld = gamepad2.x;

        boolean ready = flywheelReady();

        // Base powers
        double intakePower = intakeOn ? 1.0 : 0.0;
        double rw1Power    = rw1On ? 1.0 : 0.0;
        double rw2Power    = rw2On ? 1.0 : 0.0;

        if (unjamHeld) {
            intakePower = 0.0;
            rw1Power = -0.25;
            rw2Power = -0.25;
        } else {

            // Not ready gating
            if (flywheelOn && !ready) {
                if (intakeOn) intakePower = INTAKE_HOLD_PWR;
                rw1Power = 0.0;
                rw2Power = RW2_GATE_HOLD_PWR;
            }

            // Manual pulse overrides RW2 and pauses RW1
            if (rw2PulseActive) {
                rw2Power = RW2_PULSE_PWR;
                rw1Power = 0.0;
                if (intakeOn) intakePower = Math.min(intakePower, INTAKE_HOLD_PWR);
            }
        }

        sI.setPower(intakePower);
        sRW1.setPower(rw1Power);
        sRW2.setPower(rw2Power);

        telemetry.addData("Manual", "I=%.2f RW1=%.2f RW2=%.2f", intakePower, rw1Power, rw2Power);
        telemetry.addData("RW2 Pulse", rw2PulseActive ? "ACTIVE" : "OFF");
        telemetry.addData("Unjam", unjamHeld ? "HELD" : "OFF");
    }

    // =========================================================
    // AUTO-SHOOT state machine (hold gamepad2 RIGHT TRIGGER)
    // - Uses RPM dip detection instead of fixed FEED_PULSE_MS
    // - Auto owns sI/sRW1/sRW2 only while trigger is held
    // - If X is held (unjam), auto yields to manual immediately
    // =========================================================
    private boolean updateAutoShootStateMachine(boolean autoShootHeld) {

        long now = System.currentTimeMillis();
        long dt  = now - autoStateStartMs;

        // Unjam always wins: yield to manual
        if (gamepad2.x) {
            if (autoState != AutoShootState.IDLE) {
                dipStartMs = -1;
                shotDetectedThisCycle = false;
                enterAutoState(AutoShootState.IDLE);
            }
            return false;
        }

        double rpmNow = getFlywheelRPM();
        boolean ready = flywheelReady();

        // Released trigger -> stop auto and yield to manual
        if (!autoShootHeld && autoState != AutoShootState.IDLE) {
            sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
            sRW1.setPower(0.0);
            sRW2.setPower(RW2_GATE_HOLD_PWR);

            dipStartMs = -1;
            shotDetectedThisCycle = false;
            enterAutoState(AutoShootState.IDLE);
            return false;
        }

        // Not held and already idle -> do nothing
        if (!autoShootHeld && autoState == AutoShootState.IDLE) {
            return false;
        }

        // Auto active
        switch (autoState) {

            case IDLE:
                sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                dipStartMs = -1;
                shotDetectedThisCycle = false;
                enterAutoState(AutoShootState.WAIT_READY);
                break;

            case WAIT_READY:
                sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                sRW1.setPower(0.0);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (ready) {
                    dipStartMs = -1;
                    shotDetectedThisCycle = false;
                    sRW2.setPower(RW2_PULSE_PWR);
                    enterAutoState(AutoShootState.FEED_UNTIL_SHOT);
                }
                break;

            case FEED_UNTIL_SHOT:
                // Feed into flywheel until dip detected or timeout
                sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                sRW1.setPower(RW1_RESTAGE_PWR);
                sRW2.setPower(RW2_PULSE_PWR);

                if (!shotDetectedThisCycle && detectShotDip(rpmNow)) {
                    shotDetectedThisCycle = true;
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoState(AutoShootState.GATE_HOLD);
                }

                if (dt >= MAX_FEED_TIME_MS) {
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoState(AutoShootState.GATE_HOLD);
                }
                break;

            case GATE_HOLD:
                sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                sRW1.setPower(RW1_RESTAGE_PWR);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= GATE_HOLD_MS) {
                    enterAutoState(AutoShootState.RESTAGE);
                }
                break;

            case RESTAGE:
                // Push next ball up, but keep RW2 gated
                sI.setPower(intakeOn ? INTAKE_RESTAGE_PWR : 0.0);
                sRW1.setPower(RW1_RESTAGE_PWR);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= RESTAGE_MS) {
                    sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                    sRW1.setPower(RW1_RESTAGE_PWR);
                    sRW2.setPower(RW2_GATE_HOLD_PWR);
                    enterAutoState(AutoShootState.WAIT_RECOVER);
                }
                break;

            case WAIT_RECOVER:
                sI.setPower(intakeOn ? INTAKE_HOLD_PWR : 0.0);
                sRW1.setPower(RW1_RESTAGE_PWR);
                sRW2.setPower(RW2_GATE_HOLD_PWR);

                if (dt >= RECOVER_MIN_MS && ready) {
                    dipStartMs = -1;
                    shotDetectedThisCycle = false;
                    sRW2.setPower(RW2_PULSE_PWR);
                    enterAutoState(AutoShootState.FEED_UNTIL_SHOT);
                }
                break;
        }

        telemetry.addData("AutoShoot", "%s", autoState);
        telemetry.addData("RPM", "%.0f", rpmNow);
        telemetry.addData("TargetRPM", "%.0f", flywheelTargetRPM);
        telemetry.addData("DipThresh", "%.0f", (flywheelTargetRPM - SHOT_DIP_RPM));
        telemetry.addData("FlywheelReady", ready);

        return true;
    }

    // =========================================================
    // Helpers
    // =========================================================
    private void enterAutoState(AutoShootState s) {
        autoState = s;
        autoStateStartMs = System.currentTimeMillis();
    }

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
