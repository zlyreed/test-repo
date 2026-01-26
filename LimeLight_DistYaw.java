// FTC TeleOp snippet: Limelight distance-to-AprilTag + robot angle (yaw)
//
// What you get:
// 1) "bearingToTagDeg"  = Limelight tx (left/right angle to the tag in degrees)
// 2) "distanceMeters"   = Limelight botposeAvgDist (distance estimate; see note below)
// 3) "robotYawDeg"      = robot heading from IMU (this is the robot’s angle)
//
// NOTE about distance:
// - result.getBotposeAvgDist() is a Limelight-provided distance estimate (in meters).
// - It is NOT strictly “distance to *this* tag” if multiple tags are visible; it’s an average.
//   If you need per-tag range, you typically compute from pose data (botpose + known tag pose)
//   or use a per-tag pose/range API if available in your SDK version.

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="LL Tag Distance + Robot Angle", group="Test")
public class LLTagDistanceRobotAngle extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    // Change this to the tag you care about (example: blue=20, red=24)
    private static final int TARGET_TAG_ID = 20;

    @Override
    public void runOpMode() {
        // IMU init (adjust hub orientation to match your robot)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight init
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(0); // your AprilTag pipeline index
        limelight.start();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Robot angle (heading) from IMU
            double robotYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addData("Robot Yaw (deg)", "%.1f", robotYawDeg);
                telemetry.addLine("Limelight: no valid result");
                telemetry.update();
                continue;
            }

            // Fiducials = AprilTags
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addData("Robot Yaw (deg)", "%.1f", robotYawDeg);
                telemetry.addLine("AprilTags: none detected");
                telemetry.update();
                continue;
            }

            // Find the tag we care about
            LLResultTypes.FiducialResult target = null;
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == TARGET_TAG_ID) {
                    target = f;
                    break;
                }
            }

            telemetry.addData("Robot Yaw (deg)", "%.1f", robotYawDeg);

            if (target == null) {
                telemetry.addData("Target Tag", "%d not in view", TARGET_TAG_ID);
                telemetry.update();
                continue;
            }

            // Angle to tag (bearing) from Limelight camera crosshair
            // +tx = tag is to the RIGHT, -tx = tag is to the LEFT
            double bearingToTagDeg = target.getTargetXDegrees();

            // Distance estimate (meters)
            double distanceMeters = result.getBotposeAvgDist();

            telemetry.addData("Target Tag ID", TARGET_TAG_ID);
            telemetry.addData("Bearing to Tag (deg)", "%.1f", bearingToTagDeg);
            telemetry.addData("Distance (m)", "%.2f", distanceMeters);

            telemetry.update();
        }

        limelight.stop();
    }
}
