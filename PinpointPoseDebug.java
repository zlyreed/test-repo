package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// TODO: Replace with the actual Pinpoint driver import from your project/library.
// Example placeholders:
// import com.gobilda.pinpoint.Pinpoint;
// import com.gobilda.pinpoint.Pose2D;

@TeleOp(name="Pinpoint Pose Debug", group="Debug")
public class PinpointPoseDebug extends LinearOpMode {

    // --- CHANGE THESE (measured on your robot) ---
    // Units here: inches
    private static final double FORWARD_WHEEL_Y_OFFSET_IN = 0.0; // +left, -right
    private static final double LATERAL_WHEEL_X_OFFSET_IN  = 0.0; // +forward, -back

    // Example target point on field or your own reference (in same coordinate frame you’re using)
    private static final double TARGET_X_IN = 60.0;
    private static final double TARGET_Y_IN = 24.0;

    @Override
    public void runOpMode() {

        // TODO: Create Pinpoint instance using your actual driver class.
        // Common pattern is hardwareMap + device name:
        // Pinpoint pinpoint = new Pinpoint(hardwareMap, "pinpoint");
        Object pinpoint = initPinpoint(); // placeholder

        // ---- CONFIGURE PINPOINT (replace these with your library’s calls) ----
        // 1) Tell it which encoder is forward vs lateral (if required)
        // 2) Set wheel offsets (for 2-wheel correction)
        // 3) Set unit scale (wheel diameter / distance-per-tick / scale factor)
        configurePinpoint(pinpoint);

        telemetry.addLine("Pinpoint ready. Keep robot still for 2–3 seconds after init.");
        telemetry.update();

        waitForStart();

        // Reset pose to local origin at match start
        resetPose(pinpoint, 0.0, 0.0, 0.0);

        while (opModeIsActive()) {

            // Update/read newest Pinpoint data (some libraries require pinpoint.update())
            update(pinpoint);

            Pose pose = getPose(pinpoint); // x,y,headingDeg

            double distFromStart = Math.hypot(pose.x, pose.y);
            double distToTarget  = Math.hypot(TARGET_X_IN - pose.x, TARGET_Y_IN - pose.y);

            telemetry.addData("x (in)", "%.2f", pose.x);
            telemetry.addData("y (in)", "%.2f", pose.y);
            telemetry.addData("heading (deg)", "%.1f", pose.headingDeg);

            telemetry.addData("dist from start (in)", "%.2f", distFromStart);
            telemetry.addData("dist to target (in)", "%.2f", distToTarget);

            telemetry.update();
        }
    }

    // -----------------------------
    // Minimal “Pose” container
    // -----------------------------
    static class Pose {
        final double x, y, headingDeg;
        Pose(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    // -----------------------------
    // PLACEHOLDER FUNCTIONS
    // Replace internals with your actual Pinpoint driver calls.
    // -----------------------------
    private Object initPinpoint() {
        // TODO: return new Pinpoint(hardwareMap, "pinpoint");
        return new Object();
    }

    private void configurePinpoint(Object pp) {
        // TODO: Examples (replace with your library):
        // pp.set2WheelOffsets(FORWARD_WHEEL_Y_OFFSET_IN, LATERAL_WHEEL_X_OFFSET_IN);
        // pp.setUnitsInches();
        // pp.setEncoderDirections(/* forward sign */, /* lateral sign */);
        //
        // If your library uses wheel diameter & ticks/rev:
        // pp.setWheelDiameterInches(odometryWheelDiameter);
        // pp.setTicksPerRev(encoderTicksPerRev);

        // Keep these measured offsets written somewhere students can find.
    }

    private void resetPose(Object pp, double xIn, double yIn, double headingDeg) {
        // TODO: pp.resetPose(xIn, yIn, headingDeg);
    }

    private void update(Object pp) {
        // TODO: pp.update();
        // Some libraries update automatically on read.
    }

    private Pose getPose(Object pp) {
        // TODO: Replace with your library’s getter:
        // Pose2D p = pp.getPose();
        // return new Pose(p.xInches, p.yInches, p.headingDegrees);

        // Placeholder:
        return new Pose(0.0, 0.0, 0.0);
    }
}
