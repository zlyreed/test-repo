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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

// Commented out: Predicted RPM from exponential model: y = A * B^x
// Commented out: additional adjustment for aim assist

@TeleOp(name = "TeleOpAimPoseRPM_speedFunc", group = "Robot")
public class TeleOpAimPoseRPM_speedFunc extends LinearOpMode {

    // =========================
    // Hardware
    // =========================
    private DcMotor mFL, mFR, mBL, mBR;
    private DcMotorEx mFW; // still mapped if your robot config includes it (we're not controlling it here)
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
    // DECODE goal AprilTag center height above TILE surface:
    // 38.75 in (goal top) - 4.5 in = 34.25 in
    private static final double GOAL_TAG_CENTER_HEIGHT_IN = 29.49; // measured in the field

    // Measure these on YOUR robot:
    private static final double LL_LENS_HEIGHT_IN = 14.5;   // lens center height above tile (measured; original=12 inches); may need adjust the regression
    private static final double LL_MOUNT_ANGLE_DEG = 0;  // camera pitch up from horizontal
    private static final double LL_LATERAL_OFFSET_IN = 4.5; // camera from the center of the robot (lateral; Y axis (Y+=left): -6.5 inches)

    // -------- Goal tag field poses (same units/frame as Limelight botpose) --------
    // Fill these with the correct DECODE field coordinates that match Limelight's coordinate system.
    // If Limelight botpose is in METERS (common), keep these in meters.
    // Source: community field map post (verify your LL coordinate frame matches).
    // Tag 20 (Blue): x=-1.482, y=-1.413, z=0.749 (in meters) / x=-58.35, y=-55.63, z=29.49 (in inches)
    // Tag 24 (Red):  x=-1.482, y= 1.413, z=0.749 (in meters) /x=-58.35, y=55.63, z=29.49 (in inches);
  /* private static final double TAG20_X = -1.482;
    private static final double TAG20_Y = -1.413;
    private static final double TAG20_Z =  0.749;

    private static final double TAG24_X = -1.482;
    private static final double TAG24_Y =  1.413;
    private static final double TAG24_Z =  0.749;
*/
    // Unit conversion (if TAG coords & botpose are meters)
    private static final double M_TO_IN = 39.3700787;

    // -------- Flywheel speed and prediction model (distanceIn inches -> rpm) --------
    // Linear regression: y=16.57766x+2602.65546
    private static final double RPM_SLOPE = 16.57766;
    private static final double RPM_INTERCEPT = 2602.65546;

    // Exponential regression: y = 2854.83269 * 1.0039^x
    // private static final double RPM_A = 2854.83269;
    // private static final double RPM_B = 1.0039;

    private static final double PRED_RPM_MIN = 1500;
    private static final double PRED_RPM_MAX = 5800;

    private static final double MANUAL_RPM_UP = 3800;  // desired speed for near shooting
    private static final double MANUAL_RPM_DOWN = 5000;  // desired speed for far shooting
    private double flywheelTargetRPM = MANUAL_RPM_UP;    // start guess
    private double lastPredictedRPM = MANUAL_RPM_UP;  // fallback if limelight is not avaible.
    private double manualFallbackRPM = MANUAL_RPM_UP;


    // ---------------- LED light -------------------
    private static final double LED_GREEN_POS = 0.5; // Green
    private static final double LED_RED_POS = 0.277; // Red
    private static final double LED_OFF_POS = 0.0;; // off (try 1 if 0 doesn't work);
    private static final double LED_RPM_TOL = 150;  // speed tolerance

    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID; // toggle between blue/red
    private boolean flywheelOn = false;

   
    // inital states for intake, ramp wheel 1 and ramp wheel 2
    private boolean intakeOn = false;
    private boolean rw1On = false;
    private boolean rw2On = false;
    private boolean unjamOn = false;

    // RW2 pulse settings
    private static final long RW2_PULSE_MS = 150;       // start 50ms; tune down or up later
    private static final double RW2_PULSE_PWR = 1.0;

    // Pulse state
    private boolean rw2PulseActive = false;
    private long rw2PulseStartMs = 0;
    private static final boolean PAUSE_RW1_DURING_PULSE = true; // Optional: pause RW1 during RW2 pulse (recommended)

    //Other states    
    private static final double INTAKE_HOLD_PWR = 0.18; // Intake hold power to prevent balls falling out while waiting (tune 0.10–0.30)
    private static final double RW2_GATE_HOLD_PWR = -0.08; // RW2 gate (tiny reverse) to prevent early touch (tune -0.05 to -0.12; set 0.0 if it jams)    

    // For field-centric drive
    private double initYawDeg = 0;

    @Override
    public void runOpMode() {

        initHardware();

        initYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Ready. Drive to position.");
        telemetry.addLine("Aim: hold gamepad1 LEFT TRIGGER for aim assist to goal tag.");
        telemetry.addLine("Goal tag: gamepad1 X=Blue(20), B=Red(24)");
        telemetry.addLine("This version: prints MT1 pose + distances + predicted RPM.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Pick which goal tag you are aiming at
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Manual drive (field-centric) + optional aim assist overlay
            driveFieldCentricWithOptionalAimAssist();

            // 3) Limelight:  trig distance + predicted RPM (if limelight data is not avaiable, press gamepad1 Up to pick near shooting RPM or gamepad1 down to pick far shooting RPM
            Double predictedRPM = updateLimelightPoseAndDistanceTelemetry(goalTagId);

            // 4) Flywheel: sets the target RPM only when you toggle ON (Toggle flywheel with gamepad2 Y); LED shows Green at predicted RPM
            applyFlywheelControl(predictedRPM);

            // 5) Intake, Ramp wheel 1&2: - Intake toggle: gamepad2 RB; Ramp Wheel 1 toggle: gamepad1 RB; Ramp Wheel 2 forward toggle: gamepad2 A
            // Hold-to-unjam override: gamepad2 X
            updateIntakeAndRampWheelControls();

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

        // Keep mapped if present; not controlling here
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

        // IMU hub mounting
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(7);  // AprilTag pipeline
        limelight.start();

        //LED light
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

            // ** If additional adjustment need
            Double tyDeg = getTyToGoalTag(goalTagId, result);
            Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);

            if (txDeg != null) {
                
                // If additional adjustment need
                // txSetpoint = atan(offset / distance)
                double txSetpointDeg = Math.toDegrees(Math.atan2(LL_LATERAL_OFFSET_IN, trigDistIn));
                // Error we want to drive to zero:
                double txErrorDeg = txDeg - txSetpointDeg;
                turnAssist = clamp(txErrorDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);

               // turnAssist = clamp(txDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
                telemetry.addData("AimAssist", "ON tx=%.1f° turn=%.2f", txDeg, turnAssist);
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
// LIMELIGHT TRIG DIST + PREDICTED RPM
// Returns predicted RPM (Double) or gamepad1 Up to pick near shooting RPM or gamepad1 down to pick far shooting RPM
// =========================================================
    private Double updateLimelightPoseAndDistanceTelemetry(int goalTagId) {

        // Feed yaw for better pose fusion
        // double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // limelight.updateRobotOrientation(yawDeg);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            // Driver can select manual RPM anytime
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            // Default predictedRPM = manual, will be overwritten if we get a valid trig distance
            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("LL", "No valid result");
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        telemetry.addData("LL Pipeline", "Index=%d Type=%s", result.getPipelineIndex(), result.getPipelineType());

        // ---- Trig distance using ty of the selected goal tag ----
        Double tyDeg = getTyToGoalTag(goalTagId, result);
        Double txDeg = getTxToGoalTag(goalTagId, result);
        if (tyDeg == null) {
            // Driver can select manual RPM anytime
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            // Default predictedRPM = manual, will be overwritten if we get a valid trig distance
            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "n/a (goal tag %d not in view)", goalTagId);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);
        if (trigDistIn == null) {
            // Driver can select manual RPM anytime
            if (gamepad1.dpad_up)   manualFallbackRPM = MANUAL_RPM_UP;
            if (gamepad1.dpad_down) manualFallbackRPM = MANUAL_RPM_DOWN;

            // Default predictedRPM = manual, will be overwritten if we get a valid trig distance
            double predictedRPM = clamp(manualFallbackRPM, PRED_RPM_MIN, PRED_RPM_MAX);

            telemetry.addData("TrigDist", "invalid (ty=%.2f°)", tyDeg);
            telemetry.addData("PredRPM (Manual!!)", "%.0f", predictedRPM);
            return predictedRPM;
        }

        // ---- Compute predicted RPM from model (overwrites manual fallback) ----
        double predictedRPM = RPM_SLOPE * trigDistIn + RPM_INTERCEPT;   // Predicted RPM from the linear model (from the experiment)
        // double predictedRPM = RPM_A * Math.pow(RPM_B, trigDistIn);  // other option: Predicted RPM from exponential model: y = A * B^x

        predictedRPM = clamp(predictedRPM, PRED_RPM_MIN, PRED_RPM_MAX);

        telemetry.addData("TrigDist","%.1f in (Tx angle= %.1f, Ty angle=%.1f°) to Goal %d", trigDistIn, txDeg, tyDeg, goalTagId);
        telemetry.addData("PredRPM", "%.0f RPM", predictedRPM);
        telemetry.addData("ManualRPM", "%.0f (Use only if goal not seen!!", manualFallbackRPM);

        return predictedRPM;
    }

// =========================================================
    /* Flywheel control:
     * - gamepad2 Y toggles ON/OFF
     * - When toggled ON: set target RPM = latest predicted RPM (or fallback)
     * - While ON: runs motor using setVelocity()
     */
// =========================================================

    private void applyFlywheelControl(Double predictedRPM) {

        // Read current RPM once (used for telemetry + LED)
        double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;

        // Track last good prediction for fallback
        if (predictedRPM != null) {
            lastPredictedRPM = predictedRPM;
        }

        // Toggle flywheel with gamepad2 Y (only changes state/target)
        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                double target = (predictedRPM != null) ? predictedRPM : lastPredictedRPM;
                flywheelTargetRPM = clamp(target, PRED_RPM_MIN, PRED_RPM_MAX);
            }
        }

        // Apply outputs EVERY loop
        if (!flywheelOn) {
            mFW.setPower(0.0);
            led.setPosition(LED_OFF_POS);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            telemetry.addData("PredRPM(last)", "%.0f", lastPredictedRPM);
            return;
        }

        // Flywheel ON: keep commanding velocity
        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);

        // Flywheel is ready: LED green only on when the flywheel is within a good speed range, otherwise OFF
        boolean atSpeed = Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL;
        led.setPosition(atSpeed ? LED_GREEN_POS : LED_OFF_POS);

        telemetry.addData("Flywheel", "ON  Target=%.0f  Current=%.0f", flywheelTargetRPM, currentRPM);
        telemetry.addData("PredRPM(last)", "%.0f", lastPredictedRPM);
    }


    // =====================Intake, Rampwheel 1 &2 control:=============================
    /* - Intake toggle: gamepad2 RB
     * - Ramp Wheel 1 toggle: gamepad1 RB
     * - Ramp Wheel 2 forward toggle: gamepad2 A; pulse move: gamepad 2, left bumper
     * - Hold-to-unjam override: gamepad2 X

    - Keep intake running at a low “HOLD” power when flywheel is ON but not ready (prevents balls falling out).
    - Pause RW1 during “not ready” (removes stack pressure that causes early touch).
    - Use RW2 as a gate: apply a tiny reverse hold power when flywheel is ON but not ready (prevents the top ball from creeping into the flywheel).
    - RW2 pulse on gamepad2 LEFT BUMPER still works.
    
    Priority order:
    - Unjam (X held) overrides everything
    - RW2 pulse overrides gate/toggles
    - Otherwise, not-ready gating behavior applies when flywheelOn && !flywheelReady     
     */
    // ====================================================================================
    
private void updateIntakeAndRampWheelControls() {

    long now = System.currentTimeMillis();

    // ===== Flywheel ready check (matches your LED logic) =====
    double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;
    boolean flywheelReady = flywheelOn && (Math.abs(currentRPM - flywheelTargetRPM) <= LED_RPM_TOL);

    // ===================== TOGGLES (buttons) =====================

    // Intake toggle: gamepad2 RB
    if (gamepad2.rightBumperWasPressed()) {
        intakeOn = !intakeOn;
    }

    // Ramp Wheel 1 toggle: gamepad1 RB
    if (gamepad1.rightBumperWasPressed()) {
        rw1On = !rw1On;
    }

    // Ramp Wheel 2 toggle: gamepad2 A
    if (gamepad2.aWasPressed()) {
        rw2On = !rw2On;
    }

    // ===================== RW2 PULSE (gamepad2 LEFT BUMPER) =====================
    if (gamepad2.leftBumperWasPressed()) {
        rw2PulseActive = true;
        rw2PulseStartMs = now;
    }

    // pulse timeout
    if (rw2PulseActive && (now - rw2PulseStartMs >= RW2_PULSE_MS)) {
        rw2PulseActive = false;
    }

    // ===================== UNJAM (hold X) =====================
    boolean unjamHeld = gamepad2.x;  // hold-to-unjam

    // ===================== BASE OUTPUTS (from toggles) =====================
    double intakePower = intakeOn ? 1.0 : 0.0;
    double rw1Power    = rw1On ? 1.0 : 0.0;
    double rw2Power    = rw2On ? 1.0 : 0.0;

    // ===================== PRIORITY LOGIC =====================

    // Priority 1: UNJAM overrides everything
    if (unjamHeld) {
        // You can choose to reverse intake slightly too, but safest is off:
        intakePower = 0.0;
        rw1Power = -0.25;
        rw2Power = -0.25;
    }
    else {
        // --- If flywheel is ON but NOT ready: prevent "early touch" ---
        // Keep intake gently ON so balls don't fall out,
        // stop RW1 to remove stack pressure,
        // gate RW2 with tiny reverse so top ball can't creep into flywheel.
        if (flywheelOn && !flywheelReady) {

            if (intakeOn) intakePower = INTAKE_HOLD_PWR;  // gentle hold (instead of full push)
            rw1Power = 0.0;                               // remove stack pressure
            rw2Power = RW2_GATE_HOLD_PWR;                 // gate at the top (set 0.0 if it jams)
        }

        // Priority 2: RW2 pulse overrides gate/toggle (used for staging or shooting)
        if (rw2PulseActive) {
            rw2Power = RW2_PULSE_PWR;

            // Strongly recommended: pause RW1 during pulse so the 2nd ball doesn't creep
            if (PAUSE_RW1_DURING_PULSE) {
                rw1Power = 0.0;
            }

            // Optional: keep intake in HOLD during pulse too (reduces pressure)
            if (intakeOn) {
                intakePower = Math.min(intakePower, INTAKE_HOLD_PWR);
            }
        }
    }

    // ===================== APPLY OUTPUTS ONCE =====================
    sI.setPower(intakePower);
    sRW1.setPower(rw1Power);
    sRW2.setPower(rw2Power);

    // ===================== TELEMETRY =====================
    telemetry.addData("FlywheelReady", flywheelReady);
    telemetry.addData("IntakePwr", "%.2f", intakePower);
    telemetry.addData("RW1Pwr", "%.2f", rw1Power);
    telemetry.addData("RW2Pwr", "%.2f", rw2Power);
    telemetry.addData("RW2 Pulse", rw2PulseActive ? "ACTIVE" : "OFF");
    telemetry.addData("Unjam", unjamHeld ? "HELD" : "OFF");
}

    
    


    // =========================================================
    // Utility and Helper Functions
    // =========================================================

    // *********** make sure the value within a defined range (e.g., do not exceed the max achievable speed for flywheel)
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    //*********Returns tx (horizontal angle) to the chosen goal tag, or null if not detected.
    private Double getTxToGoalTag(int desiredId, LLResult result) {
        // LLResult result = limelight.getLatestResult();
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


    //************** Returns ty (vertical angle) to the chosen goal tag, or null if not detected.
    // * Uses the LLResult you already fetched this loop (preferred), so you don't call getLatestResult() twice.
    private Double getTyToGoalTag(int desiredId, LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == desiredId) {
                return f.getTargetYDegrees(); // ty in degrees
            }
        }
        return null;
    }

    //****************** Trig distance estimate to the goal tag center (inches) using camera geometry.
    /* tan(a1 + a2) = (h2 - h1) / d
     * d = (h2 - h1) / tan(a1 + a2)
     *
     * where:
     *  h2 = GOAL_TAG_CENTER_HEIGHT_IN  (tag center height above tile)
     *  h1 = LL_LENS_HEIGHT_IN          (camera lens height above tile)
     *  a1 = LL_MOUNT_ANGLE_DEG         (camera pitch up from horizontal)
     *  a2 = tyDeg                      (Limelight vertical angle to target)
     *
     * Returns null if the math becomes invalid (e.g., tan ~ 0).
     */
    private Double trigDistanceToGoalInchesFromTy(double tyDeg) {
        // Total vertical angle from horizontal to the target
        double totalAngleDeg = LL_MOUNT_ANGLE_DEG + tyDeg;

        // Convert to radians for tan()
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        // Height difference (inches)
        double heightDiffIn = GOAL_TAG_CENTER_HEIGHT_IN - LL_LENS_HEIGHT_IN;

        // Avoid divide-by-zero / crazy values when angle is near 0 deg
        double tanVal = Math.tan(totalAngleRad);
        if (Math.abs(tanVal) < 1e-6) return null;

        // Distance along the floor plane (inches)
        double dIn = heightDiffIn / tanVal;

        // If the angle/geometry produced a negative distance, treat as invalid
        if (dIn <= 0) return null;
        return dIn;
    }

}
