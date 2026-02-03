
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

// worked for the Lotus Robot (changed the file name to "TeleOpAimPoseRPM.java")


@TeleOp(name = "TeleOpAimPoseRPM", group = "Robot")
public class TeleOpAimPoseRPM extends LinearOpMode {

    // =========================
    // Hardware
    // =========================
    private DcMotor mFL, mFR, mBL, mBR;
    private DcMotorEx mFW;
    private CRServo sI, sRW1, sRW2;
    private IMU imu;
    private Limelight3A limelight;

    // =========================
    // Constants (tune)
    // =========================
    private static final double TICKS_PER_REV = 28;       // change to your motor spec!
    private static final int BLUE_GOAL_TAG_ID = 20;          // change if needed
    private static final int RED_GOAL_TAG_ID  = 24;          // change if needed

    // Aim assist tuning
    private static final double AIM_KP = 0.02;               // turning gain (deg -> turn power)
    private static final double AIM_MAX_TURN = 0.35;         // clamp turn power

    // Flywheel tuning
    private static final double RPM_STEP = 150;              // change per button press
    private static final double RPM_MIN = 1500;
    private static final double RPM_MAX = 6000;

    // =========================
    // Debug: DECODE Goal AprilTag Geometry
    // =========================

   // Center height of Blue (20) / Red (24) goal AprilTags above TILE surface
   // 38.75 in (panel top) - 4.5 in (tag center offset)
    private static final double GOAL_TAG_CENTER_HEIGHT_IN = 34.25;

   // Measure these on YOUR robot
   private static final double LL_LENS_HEIGHT_IN = 10.5;     // camera lens center above tile??
   private static final double LL_MOUNT_ANGLE_DEG = 0;    // camera pitch up from horizontal

   // =========================
   // DECODE Goal AprilTag poses in "standard FTC coordinates" (meters)
   // Source: community field map post (verify your LL coordinate frame matches).  :contentReference[oaicite:2]{index=2}
   // Tag 20 (Blue): x=-1.482, y=-1.413, z=0.749 (in meters) / x=-58.35, y=-55.63, z=29.49 (in inches)
   // Tag 24 (Red):  x=-1.482, y= 1.413, z=0.749 (in meters) /x=-58.35, y=55.63, z=29.49 (in inches); 
   // =========================
    
    private static final double TAG20_X_M = -1.482;
    private static final double TAG20_Y_M = -1.413;
    private static final double TAG20_Z_M =  0.749;
    
    private static final double TAG24_X_M = -1.482;
    private static final double TAG24_Y_M =  1.413;
    private static final double TAG24_Z_M =  0.749;
    
    // Meters to inches
    private static final double M_TO_IN = 39.3700787;


    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID;                // toggle between blue/red
    private boolean flywheelOn = false;
    private double flywheelTargetRPM = 4200;                 // start guess

    // For field-centric drive
    private double initYawDeg = 0;

    @Override
    public void runOpMode() {

        initHardware();

        initYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Ready. Drive to position.");
        telemetry.addLine("Aim: hold gamepad1 LEFT TRIGGER for aim assist to goal tag.");
        telemetry.addLine("Goal tag: gamepad1 X=Blue, B=Red");
        telemetry.addLine("Flywheel: gamepad2 Y toggle ON/OFF, DpadUp/Down change RPM");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Pick which goal tag you are aiming at
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Manual drive (field-centric) + optional aim assist overlay
            driveFieldCentricWithOptionalAimAssist();
          
            // ************* Debug *****************
            LLResult result = limelight.getLatestResult();
            debugPrintAllTags(result);            
          
            // 3) Flywheel RPM tuning
            updateFlywheelRPMControls();
            applyFlywheelControl();

            // 4) Limelight pose telemetry (robot location & yaw)
            updateLimelightPoseTelemetry();

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

        // IMU hub mounting
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(0);  // AprilTag pipeline
        limelight.start();
    }

    // =========================================================
    // 1) DRIVE + AIM ASSIST
    //    - Manual field-centric drive is always active
    //    - If driver holds LEFT TRIGGER, we add "turn" correction
    //      to center the chosen goal tag (goalTagId).
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

        if (aimAssistEnabled) {
            Double txDeg = getTxToGoalTag(goalTagId);
            if (txDeg != null) {
                // If tag is to the right (positive tx), turn right (positive turn)
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

    /**
     * Returns tx (horizontal angle) to the chosen goal tag, or null if not detected.
     * Uses FiducialResults list from LLResult. :contentReference[oaicite:1]{index=1}
     */
    private Double getTxToGoalTag(int desiredId) {
        LLResult result = limelight.getLatestResult();
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

    // =========================================================
    // 2) FLYWHEEL RPM: toggle + adjust RPM
    //    - gamepad2 Y toggles flywheel ON/OFF
    //    - gamepad2 dpad up/down changes RPM target
    //    - uses DcMotorEx.setVelocity(ticksPerSecond)
    // =========================================================
    private void updateFlywheelRPMControls() {

        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;
        }

        if (gamepad2.dpad_up) {
            flywheelTargetRPM += RPM_STEP;
        } else if (gamepad2.dpad_down) {
            flywheelTargetRPM -= RPM_STEP;
        }

        flywheelTargetRPM = clamp(flywheelTargetRPM, RPM_MIN, RPM_MAX);
    }

    private void applyFlywheelControl() {
        double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;

        if (!flywheelOn) {
            mFW.setPower(0.0);
            telemetry.addData("Flywheel", "OFF (RPM=%.0f)", currentRPM);
            return;
        }

        // Convert RPM -> ticks/sec for setVelocity
        double targetTicksPerSec = flywheelTargetRPM * TICKS_PER_REV / 60.0;
        mFW.setVelocity(targetTicksPerSec);

        telemetry.addData("Flywheel", "ON Target=%.0f RPM  Current=%.0f RPM",
                flywheelTargetRPM, currentRPM);
    }

    // =========================================================
    // 3) LIMELIGHT POSE TELEMETRY (robot location + yaw)
    //    - MegaTag2 is recommended if you feed robot yaw in
    //    - result.getBotpose_MT2() gives Pose3D :contentReference[oaicite:2]{index=2}
    // =========================================================
    private void updateLimelightPoseTelemetry() {

        // Tell Limelight which way robot is facing (for MegaTag2 fusion)
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDeg); // per Limelight FTC docs :contentReference[oaicite:3]{index=3}

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("LL Pose", "No valid result");
            return;
        }

        // Use MegaTag2 pose if available
        Pose3D pose = result.getBotpose_MT2(); // :contentReference[oaicite:4]{index=4}
        if (pose == null) {
            // fallback to MT1
            pose = result.getBotpose(); // :contentReference[oaicite:5]{index=5}
        }

        if (pose == null) {
            telemetry.addData("LL Pose", "Pose is null (check Full 3D + camera pose in LL UI)");
            telemetry.addData("TagCount", result.getBotposeTagCount());
            return;
        }

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double z = pose.getPosition().z;

        // Pose3D rotation access varies by SDK, so we also show IMU yaw as "robot heading"
        telemetry.addData("LL Position (field)", "x=%.2f y=%.2f z=%.2f", x, y, z);
        telemetry.addData("LL TagCount/AvgDist", "%d / %.2f m",
                result.getBotposeTagCount(), result.getBotposeAvgDist());
    }

    // =========================================================
    //   DEBUG
    //    - Ouput pipeline information
    //    - Output Fiducial (AprilTag) information
    // =========================================================
  
  private void debugPrintAllTags(LLResult result) {
    if (result == null || !result.isValid()) {
        telemetry.addData("LL", "No valid result");
        return;
    }

    // *******Pipeline debug**********
    telemetry.addData(
            "LL Pipeline",
            "Index=%d  Type=%s",
            result.getPipelineIndex(),
            result.getPipelineType()
    );

    // *********Fiducial (AprilTag) debug**********
    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

    if (fiducials == null || fiducials.isEmpty()) {
        telemetry.addData("LL Tags", "none");
        return;
    }

    StringBuilder sb = new StringBuilder();
    for (LLResultTypes.FiducialResult f : fiducials) {
        sb.append("ID ")
          .append(f.getFiducialId())
          .append(String.format("(tx=%.1f°) ", f.getTargetXDegrees()));
    }

    telemetry.addData("LL Tags", sb.toString());
}



    //================================================
    // Limelight diagnostics for DECODE goal AprilTags (20 / 24):
    // Outputs:
    // - Pipeline index/type
    // - Robot pose from MT2 and MT1 (x,y,z); distances from robot to goal (Blue or Red); TagCount and AvgDist
    // - Per-tag tx / ty (in degrees)
    // - Trig-based distance (2D) to selected goal tag (20 or 24)-- using ty angle
    //================================================

   private void updateLimelightDiagnosticsTelemetry(int goalTagId) {

    // ---------------------------------------------------------
    // 0) Feed robot yaw to Limelight (required for MegaTag2)
    // ---------------------------------------------------------
    double yawDeg = imu.getRobotYawPitchRollAngles()
                       .getYaw(AngleUnit.DEGREES);
    limelight.updateRobotOrientation(yawDeg);

    // ---------------------------------------------------------
    // 1) Get latest Limelight result
    // ---------------------------------------------------------
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
        telemetry.addData("LL", "No valid result");
        return;
    }

    // ---------------------------------------------------------
    // 2) Pipeline info (sanity check)
    // ---------------------------------------------------------
    telemetry.addData(
            "LL Pipeline",
            "Index=%d Type=%s",
            result.getPipelineIndex(),
            result.getPipelineType()
    );

    // ---------------------------------------------------------
    // 3) Robot pose estimates (field frame) and distances from robot to Goal (Blue or Red)
    // ---------------------------------------------------------
    // Helper: choose goal tag position (meters) based on goalTagId
    double goalX_m, goalY_m, goalZ_m;
    if (goalTagId == BLUE_GOAL_TAG_ID) {
        goalX_m = TAG20_X_M; goalY_m = TAG20_Y_M; goalZ_m = TAG20_Z_M;
    } else { // RED
        goalX_m = TAG24_X_M; goalY_m = TAG24_Y_M; goalZ_m = TAG24_Z_M;
    }

    // ---- MT2 distance to goal ----
    Pose3D mt2 = result.getBotpose_MT2();
    telemetry.addData(
            "LL MT2 Pose",
            (mt2 != null)
                    ? String.format("x=%.2f y=%.2f z=%.2f",
                        mt2.getPosition().x,
                        mt2.getPosition().y,
                        mt2.getPosition().z)
                    : "null"
    );

    if (mt2 != null) {
        double dx = goalX_m - mt2.getPosition().x;
        double dy = goalY_m - mt2.getPosition().y;
        double dz = goalZ_m - mt2.getPosition().z;
    
        double distXY_m = Math.hypot(dx, dy);
        double dist3D_m = Math.sqrt(dx*dx + dy*dy + dz*dz);
    
        telemetry.addData("MT2→Goal DistXY(2D)", "%.2f m (%.1f in)", distXY_m, distXY_m * M_TO_IN);
        telemetry.addData("MT2→Goal Dist3D", "%.2f m (%.1f in)", dist3D_m, dist3D_m * M_TO_IN);
    }

    // ---- MT1 distance to goal ----
    Pose3D mt1 = result.getBotpose();
    telemetry.addData(
            "LL MT1 Pose",
            (mt1 != null)
                    ? String.format("x=%.2f y=%.2f z=%.2f",
                        mt1.getPosition().x,
                        mt1.getPosition().y,
                        mt1.getPosition().z)
                    : "null"
    );
    
    if (mt1 != null) {
        double dx = goalX_m - mt1.getPosition().x;
        double dy = goalY_m - mt1.getPosition().y;
        double dz = goalZ_m - mt1.getPosition().z;
    
        double distXY_m = Math.hypot(dx, dy);
        double dist3D_m = Math.sqrt(dx*dx + dy*dy + dz*dz);
    
        telemetry.addData("MT1→Goal DistXY(2D)", "%.2f m (%.1f in)", distXY_m, distXY_m * M_TO_IN);
        telemetry.addData("MT1→Goal Dist3D", "%.2f m (%.1f in)", dist3D_m, dist3D_m * M_TO_IN);
    }
    

    telemetry.addData(
            "LL TagCount/AvgDist",
            "%d / %.2f m",
            result.getBotposeTagCount(),
            result.getBotposeAvgDist()
    );

    // ---------------------------------------------------------
    // 4) Per-tag measurements (tx / ty in degree)
    // ---------------------------------------------------------
    List<LLResultTypes.FiducialResult> fiducials =
            result.getFiducialResults();

    if (fiducials == null || fiducials.isEmpty()) {
        telemetry.addData("LL Fiducials", "none");
        telemetry.addData("TrigDist", "n/a (no tags)");
        return;
    }

    Double tyDegForGoal = null;

    for (LLResultTypes.FiducialResult f : fiducials) {
        int id = f.getFiducialId();
        double tx = f.getTargetXDegrees();
        double ty = f.getTargetYDegrees();

        telemetry.addData(
                "Tag " + id,
                "tx=%.1f° ty=%.1f°",
                tx, ty
        );

        if (id == goalTagId) {
            tyDegForGoal = ty;
        }
    }

    // ---------------------------------------------------------
    // 5) Trig-based (2D) distance to selected goal tag (20 or 24)
    // ---------------------------------------------------------
    if (tyDegForGoal == null) {
        telemetry.addData(
                "TrigDist",
                "n/a (goal tag %d not in view)",
                goalTagId
        );
        return;
    }

    // angle from floor to goal center
    double angleToGoalDeg = LL_MOUNT_ANGLE_DEG + tyDegForGoal;
    double angleToGoalRad = Math.toRadians(angleToGoalDeg);

    double heightDiffIn = GOAL_TAG_CENTER_HEIGHT_IN - LL_LENS_HEIGHT_IN;

    // Guard against tan(0) or near-0
    if (Math.abs(Math.tan(angleToGoalRad)) < 1e-6) {
        telemetry.addData("TrigDist", "invalid (angle too small)");
        return;
    }

    double distanceIn =
            heightDiffIn / Math.tan(angleToGoalRad);

    telemetry.addData(
            "TrigDist(2D) → Goal %d",
            "%.1f in  (angle=%.1f°)",
            goalTagId,
            distanceIn,
            angleToGoalDeg
    );
}



    

    // =========================================================
    // Utility
    // =========================================================
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
