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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

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
    private static final double LL_LENS_HEIGHT_IN = 12;   // lens center height above tile (measured)
    private static final double LL_MOUNT_ANGLE_DEG = 0;  // camera pitch up from horizontal

    // -------- Goal tag field poses (same units/frame as Limelight botpose) --------
    // Fill these with the correct DECODE field coordinates that match Limelight's coordinate system.
    // If Limelight botpose is in METERS (common), keep these in meters.
    // Source: community field map post (verify your LL coordinate frame matches).  
   // Tag 20 (Blue): x=-1.482, y=-1.413, z=0.749 (in meters) / x=-58.35, y=-55.63, z=29.49 (in inches)
   // Tag 24 (Red):  x=-1.482, y= 1.413, z=0.749 (in meters) /x=-58.35, y=55.63, z=29.49 (in inches); 

    // Example placeholders (in meter; replace with your verified values):
    private static final double TAG20_X = -1.482;
    private static final double TAG20_Y = -1.413;
    private static final double TAG20_Z =  0.749;

    private static final double TAG24_X = -1.482;
    private static final double TAG24_Y =  1.413;
    private static final double TAG24_Z =  0.749;

    // Unit conversion (if TAG coords & botpose are meters)
    private static final double M_TO_IN = 39.3700787;

    // -------- Flywheel prediction model (distanceIn inches -> rpm) --------
    // Linear regression: y=16.57766x+2602.65546
    private static final double RPM_SLOPE = 16.57766;
    private static final double RPM_INTERCEPT = 2602.65546;

    // Exponential regression: y = 2854.83269 * 1.0039^x
   // private static final double RPM_A = 2854.83269;
    // private static final double RPM_B = 1.0039;

    private static final double PRED_RPM_MIN = 1500;
    private static final double PRED_RPM_MAX = 6000;

    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID; // toggle between blue/red
    private boolean flywheelOn = false;
    private double flywheelTargetRPM = 0;                 // start guess
    private double lastPredictedRPM = 4200;  // fallback if limelight is not avaible.

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

            // 3) Limelight: MT1 pose + distance-to-goal + trig distance + predicted RPM
            Double predictedRPM = updateLimelightPoseAndDistanceTelemetry(goalTagId);

            // 4) Flywheel: sets the target RPM only when you toggle ON (Toggle flywheel with gamepad2 Y)
            applyFlywheelControl(predictedRPM);
            
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
            
            // ** If additional adjustment neeed
            //Double tyDeg = getTyToGoalTag(goalTagId, result);
            //Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);
            
            if (txDeg != null) {
                
                /* If additional adjustment neeed
                // txSetpoint = atan(offset / distance)
                double txSetpointDeg = Math.toDegrees(Math.atan2(LL_LATERAL_OFFSET_IN, trigDistIn));
                // Error we want to drive to zero:
                double txErrorDeg = txDeg - txSetpointDeg;
                turnAssist = clamp(txErrorDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);    
                */
                
                turnAssist = clamp(txDeg * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);                              
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
// LIMELIGHT MT1 POSE + DISTANCES + TRIG DIST + PREDICTED RPM
// Returns predicted RPM (Double) or null if not available
// =========================================================
private Double updateLimelightPoseAndDistanceTelemetry(int goalTagId) {

    // Feed yaw for better pose fusion
    double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    limelight.updateRobotOrientation(yawDeg);

    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
        telemetry.addData("LL", "No valid result");
        return null;
    }

    telemetry.addData("LL Pipeline", "Index=%d Type=%s",
            result.getPipelineIndex(), result.getPipelineType());

    // ---- MT1 pose only ----
    Pose3D mt1 = result.getBotpose();
    if (mt1 == null) {
        telemetry.addData("LL MT1 Pose", "null");
        return null;
    }

    double rx = mt1.getPosition().x;
    double ry = mt1.getPosition().y;
    double rz = mt1.getPosition().z;

    telemetry.addData("LL MT1 Pose (field)", "x=%.2f y=%.2f z=%.2f", rx, ry, rz);

    // ---- Goal tag pose (field) ----
    double gx, gy, gz;
    if (goalTagId == BLUE_GOAL_TAG_ID) {
        gx = TAG20_X; gy = TAG20_Y; gz = TAG20_Z;
    } else {
        gx = TAG24_X; gy = TAG24_Y; gz = TAG24_Z;
    }

    double dx = gx - rx;
    double dy = gy - ry;
    double dz = gz - rz;

    double distXY = Math.hypot(dx, dy);
    double dist3D = Math.sqrt(dx*dx + dy*dy + dz*dz);

    telemetry.addData("MT1→Goal DistXY", "%.2f (%.1f in)", distXY, distXY * M_TO_IN);
    telemetry.addData("MT1→Goal Dist3D", "%.2f (%.1f in)", dist3D, dist3D * M_TO_IN);

    telemetry.addData("LL TagCount/AvgDist", "%d / %.2f m",
            result.getBotposeTagCount(), result.getBotposeAvgDist());

    // ---- Trig distance using ty of the selected goal tag ----
    Double tyDeg = getTyToGoalTag(goalTagId, result);
    if (tyDeg == null) {
        telemetry.addData("TrigDist", "n/a (goal tag %d not in view)", goalTagId);
        telemetry.addData("PredRPM", "n/a");
        return null;
    }

    Double trigDistIn = trigDistanceToGoalInchesFromTy(tyDeg);
    if (trigDistIn == null) {
        telemetry.addData("TrigDist", "invalid (ty=%.2f°)", tyDeg);
        telemetry.addData("PredRPM", "n/a");
        return null;
    }

    // Predicted RPM from your linear model
    double predictedRPM = RPM_SLOPE * trigDistIn + RPM_INTERCEPT;
    // double predictedRPM = RPM_A * Math.pow(RPM_B, trigDistIn);  // Predicted RPM from exponential model: y = A * B^x
    
    predictedRPM = clamp(predictedRPM, PRED_RPM_MIN, PRED_RPM_MAX);

    telemetry.addData(String.format("TrigDist → Goal %d", goalTagId),"%.1f in  (angle=%.1f°)", trigDistIn, tyDeg);
    telemetry.addData("PredRPM", "%.0f RPM", predictedRPM);   

    return predictedRPM;
}

// =========================================================
 /* Flywheel control:
 * - gamepad2 Y toggles ON/OFF
 * - When toggled ON: set target RPM = latest predicted RPM (or fallback)
 * - While ON: runs motor using setVelocity()
 * Pass predictedRPM from Limelight (may be null).
*/
// =========================================================
    
private void applyFlywheelControl(Double predictedRPM) {

    // Track last good prediction for fallback
    if (predictedRPM != null) {
        lastPredictedRPM = predictedRPM;
    }

    // Toggle flywheel with gamepad2 Y
    if (gamepad2.yWasPressed()) {
        flywheelOn = !flywheelOn;

        if (flywheelOn) {
            // Pick target at the moment we turn ON
            double target = (predictedRPM != null) ? predictedRPM : lastPredictedRPM;
            flywheelTargetRPM = clamp(target, PRED_RPM_MIN, PRED_RPM_MAX);
        }
    }

    // Read current RPM for telemetry
    double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;

    if (!flywheelOn) {
        mFW.setPower(0.0);
        telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
        telemetry.addData("PredRPM(last)", "%.0f", lastPredictedRPM);
        return;
    }

    // Run flywheel at target velocity
    double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
    mFW.setVelocity(targetTicksPerSec);

    telemetry.addData("Flywheel", "ON  Target=%.0f  Current=%.0f", flywheelTargetRPM, currentRPM);
    telemetry.addData("PredRPM(last)", "%.0f", lastPredictedRPM);
}

    
    // =========================================================
    // Utility and Helper Functions
    // =========================================================
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

     /**
     * Returns tx (horizontal angle) to the chosen goal tag, or null if not detected.
     */
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


    
    /**
 * Returns ty (vertical angle) to the chosen goal tag, or null if not detected.
 * Uses the LLResult you already fetched this loop (preferred), so you don't call getLatestResult() twice.
 */
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

/**
 * Trig distance estimate to the goal tag center (inches) using camera geometry.
 *
 * tan(a1 + a2) = (h2 - h1) / d
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
