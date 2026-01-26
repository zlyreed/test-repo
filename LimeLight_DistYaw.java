// FTC TeleOp snippet: Drive to position + Limelight field pose (distance to AprilTag + yaw)
//
// What you get:
// 1) "bearingToTagDeg"  = Limelight tx (left/right angle to the tag in degrees)
// 2) "distanceMeters"   = Limelight botposeAvgDist (distance estimate; see note below)
// 3) "fieldYawDeg"      = robot yaw from Limelight botpose (field coordinate system)
// 4) "robotYawDeg"      = robot heading from IMU (for comparison)
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="LL Tag Distance + Robot Angle", group="Test")
public class LLTagDistanceRobotAngle extends LinearOpMode {

    private DcMotor mFL;
    private DcMotor mFR;
    private DcMotor mBL;
    private DcMotor mBR;
    private Limelight3A limelight;
    private IMU imu;

    // Change this to the tag you care about (example: blue=20, red=24)
    private static final int TARGET_TAG_ID = 20;

    @Override
    public void runOpMode() {
        // Drive motors (mecanum). Update names to match your config.
        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        // Motor directions: keep consistent with your robot wiring
        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

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
            // Manual drive: left stick = translate, right stick = rotate
            driveRobot();

            // Robot angle (heading) from IMU
            double robotYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addData("Robot Yaw (deg)", "%.1f", robotYawDeg);
                telemetry.addLine("Limelight: no valid result");
                telemetry.update();
                continue;
            }

            // Robot field pose from Limelight (MegaTag2 preferred)
            Pose3D pose = result.getBotpose_MT2();
            if (pose == null) {
                pose = result.getBotpose();
            }

            // Fiducials = AprilTags
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addData("Robot Yaw (deg)", "%.1f", robotYawDeg);
                if (pose != null) {
                    double fieldYawDeg = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Field Yaw (deg)", "%.1f", fieldYawDeg);
                    telemetry.addData("Field Pose (m)", "x=%.2f y=%.2f z=%.2f",
                            pose.getPosition().x, pose.getPosition().y, pose.getPosition().z);
                }
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
                if (pose != null) {
                    double fieldYawDeg = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Field Yaw (deg)", "%.1f", fieldYawDeg);
                    telemetry.addData("Field Pose (m)", "x=%.2f y=%.2f z=%.2f",
                            pose.getPosition().x, pose.getPosition().y, pose.getPosition().z);
                }
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
            if (pose != null) {
                double fieldYawDeg = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("Field Yaw (deg)", "%.1f", fieldYawDeg);
                telemetry.addData("Field Pose (m)", "x=%.2f y=%.2f z=%.2f",
                        pose.getPosition().x, pose.getPosition().y, pose.getPosition().z);
            } else {
                telemetry.addLine("Field Pose: null (check Limelight 3D settings)");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private void driveRobot() {
        double y = -gamepad1.left_stick_y; // forward
        double x = gamepad1.left_stick_x;  // strafe
        double turn = gamepad1.right_stick_x;

        double pFL = y + x + turn;
        double pFR = y - x - turn;
        double pBL = y - x + turn;
        double pBR = y + x - turn;

        double max = Math.max(Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR)));
        if (max > 1.0) {
            pFL /= max;
            pFR /= max;
            pBL /= max;
            pBR /= max;
        }

        mFL.setPower(pFL);
        mFR.setPower(pFR);
        mBL.setPower(pBL);
        mBR.setPower(pBR);
    }
}
