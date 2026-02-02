package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// -------- Limelight imports (adjust package names to your project) --------
// Many FTC Limelight examples use these (or similar) classes:
import com.limelight.Limelight3A;
import com.limelight.LLResult;
import com.limelight.results.LLTargetFiducial;

// -------- Pinpoint imports (adjust to your Pinpoint library) -------------
// Your project may use different class names. Replace these with yours.
import com.gobilda.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Decode: Pinpoint + Limelight Distance + Localization Demo")
public class DecodeDistanceAndLocalizationDemo extends OpMode {

    // ---------------------- Hardware ----------------------
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private IMU imu;

    // ---------------------- Settings ----------------------
    // Choose which “goal tags” you care about (example IDs — change to your Decode IDs)
    // If you only want Blue or Red goal tags, list them here.
    private static final int[] GOAL_TAG_IDS = new int[]{20, 24};

    // FTC field coordinates should be consistent across ALL sources:
    // - Pinpoint: whatever you configure as (0,0) and axes directions
    // - Limelight botpose: depends on which field AprilTag layout is loaded in LL
    // You MUST make them match.
    //
    // Fill these in with *Decode* AprilTag field coordinates (meters).
    // Format: tagId -> Pose2D(xMeters, yMeters, headingRad)  (tag heading usually not needed for distance)
    private static final Map<Integer, Pose2D> DECODE_TAG_POSES_METERS = new HashMap<>();
    static {
        // TODO: Replace with the official Decode tag coordinates (meters).
        // Example placeholders (DO NOT USE AS REAL):
        DECODE_TAG_POSES_METERS.put(20, new Pose2D( 1.000, 2.000, 0.0));
        DECODE_TAG_POSES_METERS.put(24, new Pose2D( 4.000, 2.000, 0.0));
    }

    // Camera geometry backup method (only if you know these)
    // Used for trig estimate from tx,ty (rough). Optional.
    private static final double CAMERA_HEIGHT_M = 0.30;   // TODO: measure
    private static final double TAG_HEIGHT_M    = 0.15;   // TODO: official tag center height
    private static final double CAMERA_PITCH_DEG = 0.0;   // TODO: camera tilt up/down (deg)

    @Override
    public void init() {
        // ------------------- Init hardware -------------------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // ------------------- Pinpoint setup -------------------
        // These calls depend on your Pinpoint library version.
        // Keep all Pinpoint-specific config here.
        configurePinpoint(pinpoint);

        // ------------------- Limelight setup ------------------
        // Start the limelight pipeline that detects AprilTags (fiducials).
        // Method names can differ slightly by LL FTC library version.
        limelight.pipelineSwitch(0); // TODO: set to your AprilTag pipeline index
        limelight.start();

        telemetry.addLine("Initialized. Press Play.");
    }

    @Override
    public void loop() {
        // ------------------- Read IMU yaw -------------------
        double imuYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // ------------------- Update Pinpoint -----------------
        // (Exact update call depends on your version)
        pinpoint.update();
        Pose2D posePinpoint = getPoseFromPinpoint(pinpoint); // meters + radians (we’ll enforce)

        // ------------------- Read Limelight result ------------
        LLResult result = limelight.getLatestResult();
        boolean hasLL = (result != null) && result.isValid();

        // (1) Robot pose from Limelight (field coordinate system)
        Pose2D poseLL = null;
        if (hasLL) {
            // MegaTag2 botpose is recommended if you feed robot yaw to LL
            // Some APIs provide: result.getBotpose_MT2() or result.getBotpose()
            // We'll assume botpose is [x,y,z,roll,pitch,yaw] in meters + degrees.
            double[] botpose = result.getBotpose_MT2(); // adjust if needed
            if (botpose != null && botpose.length >= 6) {
                double x = botpose[0];
                double y = botpose[1];
                double yawDeg = botpose[5];
                poseLL = new Pose2D(x, y, Math.toRadians(yawDeg));
            }
        }

        // (2) Direct camera->tag distance from Limelight fiducial target
        // We'll pick the best (closest / highest area) goal tag detected.
        TagMeasurement bestGoalMeas = null;
        if (hasLL) {
            bestGoalMeas = findBestGoalTagMeasurement(result, GOAL_TAG_IDS);
        }

        // ------------------- DISTANCE METHODS -------------------
        // Method A: camera -> tag 3D distance (from LL fiducial t6c / transform)
        Double distCamToTag_m = null;
        Integer seenTagId = null;
        if (bestGoalMeas != null) {
            distCamToTag_m = bestGoalMeas.distance3D_m;
            seenTagId = bestGoalMeas.tagId;
        }

        // Method B: robot -> tag planar distance using Limelight robot pose + known tag pose
        Double distRobotToTag_fromLLPose_m = null;
        if (poseLL != null && seenTagId != null && DECODE_TAG_POSES_METERS.containsKey(seenTagId)) {
            Pose2D tagPose = DECODE_TAG_POSES_METERS.get(seenTagId);
            distRobotToTag_fromLLPose_m = planarDistanceMeters(poseLL, tagPose);
        }

        // Method C: robot -> tag planar distance using Pinpoint pose + known tag pose
        Double distRobotToTag_fromPinpoint_m = null;
        if (posePinpoint != null && seenTagId != null && DECODE_TAG_POSES_METERS.containsKey(seenTagId)) {
            Pose2D tagPose = DECODE_TAG_POSES_METERS.get(seenTagId);
            distRobotToTag_fromPinpoint_m = planarDistanceMeters(posePinpoint, tagPose);
        }

        // Method D (backup): rough trig from tx/ty + heights (needs camera pitch & tag height)
        Double distTrig_m = null;
        if (bestGoalMeas != null) {
            distTrig_m = trigDistanceEstimate(bestGoalMeas.tyDeg);
        }

        // ------------------- TELEMETRY OUTPUT -------------------
        telemetry.addLine("=== Localization (Field) ===");
        if (posePinpoint != null) {
            telemetry.addData("Pinpoint Pose", "x=%.3f m, y=%.3f m, hdg=%.1f deg",
                    posePinpoint.x, posePinpoint.y, Math.toDegrees(posePinpoint.headingRad));
        } else {
            telemetry.addLine("Pinpoint Pose: (null)");
        }

        if (poseLL != null) {
            telemetry.addData("Limelight Pose", "x=%.3f m, y=%.3f m, hdg=%.1f deg",
                    poseLL.x, poseLL.y, Math.toDegrees(poseLL.headingRad));
        } else {
            telemetry.addLine("Limelight Pose: (no valid botpose)");
        }

        telemetry.addData("IMU Yaw", "%.1f deg", imuYawDeg);

        telemetry.addLine();
        telemetry.addLine("=== Goal Tag Distance ===");
        if (bestGoalMeas != null) {
            telemetry.addData("Seen Goal Tag", "ID=%d  tx=%.1f  ty=%.1f",
                    bestGoalMeas.tagId, bestGoalMeas.txDeg, bestGoalMeas.tyDeg);
        } else {
            telemetry.addLine("Seen Goal Tag: none");
        }

        telemetry.addData("A) Cam->Tag (LL 3D)", fmt(distCamToTag_m));
        telemetry.addData("B) Robot->Tag (LL pose + map)", fmt(distRobotToTag_fromLLPose_m));
        telemetry.addData("C) Robot->Tag (Pinpoint + map)", fmt(distRobotToTag_fromPinpoint_m));
        telemetry.addData("D) Robot->Tag (Trig rough)", fmt(distTrig_m));

        telemetry.update();
    }

    // --------------------------- Helpers ---------------------------

    private void configurePinpoint(GoBildaPinpointDriver pp) {
        // Keep Pinpoint configuration in one place.
        // Examples (names vary by version):
        // pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // pp.setOffsets(xOffsetMM, yOffsetMM);
        // pp.setYawScalar(1.0);
        // pp.resetPosAndIMU();
    }

    private Pose2D getPoseFromPinpoint(GoBildaPinpointDriver pp) {
        // Replace with your library’s actual pose getters.
        // Many versions provide x/y in mm and heading in radians or degrees.
        // Convert everything to meters + radians here.

        // Example pseudo-API (adjust):
        double x_mm = pp.getX();            // TODO: replace
        double y_mm = pp.getY();            // TODO: replace
        double heading_rad = pp.getHeading(); // TODO: replace (if degrees, convert!)

        return new Pose2D(x_mm / 1000.0, y_mm / 1000.0, heading_rad);
    }

    private TagMeasurement findBestGoalTagMeasurement(LLResult result, int[] goalIds) {
        // Prefer a goal tag if detected. Among matches, choose “best” (closest or largest).
        // Many LL APIs give a list of fiducials: result.getFiducialTargets()

        java.util.List<LLTargetFiducial> fiducials = result.getFiducialTargets();
        if (fiducials == null) return null;

        TagMeasurement best = null;
        for (LLTargetFiducial t : fiducials) {
            int id = t.getFiducialID();
            if (!isIn(id, goalIds)) continue;

            // tx/ty are angular offsets (deg)
            double tx = t.getTx();
            double ty = t.getTy();

            // Many LL fiducial targets provide a 3D transform “t6c” or similar.
            // If available, use it for distance. If not, you can skip 3D distance.
            // Example: double[] t6c = t.getTargetPoseCameraSpace();  // adjust name
            double dist3d = Double.NaN;
            double[] camToTarget = t.getTargetPoseCameraSpace(); // adjust if needed: [x,y,z,roll,pitch,yaw]
            if (camToTarget != null && camToTarget.length >= 3) {
                double cx = camToTarget[0];
                double cy = camToTarget[1];
                double cz = camToTarget[2];
                dist3d = Math.sqrt(cx * cx + cy * cy + cz * cz);
            }

            TagMeasurement meas = new TagMeasurement(id, tx, ty, dist3d);

            // Pick “best”: smallest 3D distance if valid, otherwise smallest |tx| as fallback
            if (best == null) {
                best = meas;
            } else {
                boolean measHasDist = !Double.isNaN(meas.distance3D_m);
                boolean bestHasDist = !Double.isNaN(best.distance3D_m);
                if (measHasDist && bestHasDist) {
                    if (meas.distance3D_m < best.distance3D_m) best = meas;
                } else if (measHasDist && !bestHasDist) {
                    best = meas;
                } else if (!measHasDist && !bestHasDist) {
                    if (Math.abs(meas.txDeg) < Math.abs(best.txDeg)) best = meas;
                }
            }
        }
        return best;
    }

    private boolean isIn(int id, int[] list) {
        for (int v : list) if (v == id) return true;
        return false;
    }

    private double planarDistanceMeters(Pose2D a, Pose2D b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private Double trigDistanceEstimate(double tyDeg) {
        // Rough estimate: distance = (tagHeight - camHeight) / tan(camPitch + ty)
        // Only meaningful if you have a good camera pitch angle + tag center height.
        double totalPitchDeg = CAMERA_PITCH_DEG + tyDeg;
        double totalPitchRad = Math.toRadians(totalPitchDeg);
        double dh = TAG_HEIGHT_M - CAMERA_HEIGHT_M;
        double tan = Math.tan(totalPitchRad);
        if (Math.abs(tan) < 1e-6) return null;
        double dist = dh / tan;
        if (dist < 0) dist = -dist;
        return dist;
    }

    private String fmt(Double meters) {
        if (meters == null) return "n/a";
        return String.format("%.3f m", meters);
    }

    // ----------------------- Small data classes -----------------------

    private static class Pose2D {
        final double x;          // meters
        final double y;          // meters
        final double headingRad; // radians
        Pose2D(double x, double y, double headingRad) {
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }
    }

    private static class TagMeasurement {
        final int tagId;
        final double txDeg;
        final double tyDeg;
        final double distance3D_m; // may be NaN if not available
        TagMeasurement(int tagId, double txDeg, double tyDeg, double distance3D_m) {
            this.tagId = tagId;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.distance3D_m = distance3D_m;
        }
    }
}
