package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

/* Notes: 
Check TODO items to make sure they fit your situation.

Quick tuning advice:
- If the robot turns the wrong direction when aiming: change turnAssist = txDeg * AIM_KP to turnAssist = -txDeg * AIM_KP
- If it oscillates/wiggles near center:increase TX_DEADBAND_DEG a bit (e.g., 1 → 2) or lower AIM_KP (e.g., 0.02 → 0.015)
- If it’s too weak/slow to center:raise AIM_KP slightly (0.02 → 0.025) or raise AIM_MAX_TURN slightly (0.35 → 0.45)

*/


@Disabled
@TeleOp(name = "TeleOp_AimAssist", group = "Robot")
public class TeleOp_AimAssist extends LinearOpMode {

    // =========================
    // Hardware
    // =========================
    private DcMotor mFL, mFR, mBL, mBR;
    private IMU imu;
    private Limelight3A limelight;

    // =========================
    // Constants 
    // =========================
    // TODO: update IDs for your field/game
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    // Aim assist tuning knobs
    private static final double AIM_KP = 0.02;        // deg -> turn power
    private static final double AIM_MAX_TURN = 0.35;  // clamp assist

    // Optional: small deadband to stop “hunting” near center
    private static final double TX_DEADBAND_DEG = 1.0;

    // =========================
    // State
    // =========================
    private int goalTagId = BLUE_GOAL_TAG_ID;

    // Field-centric offset (we treat the start heading as “field forward”)
    private double initYawDeg = 0;

    @Override
    public void runOpMode() {

        initHardware();

        // Save the heading at init as our "field forward"
        initYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Ready. Drive to position.");
        telemetry.addLine("X=Blue goal, B=Red goal, Hold LT = Aim Assist");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1) Choose which goal tag to aim at
            if (gamepad1.xWasPressed()) goalTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.bWasPressed()) goalTagId = RED_GOAL_TAG_ID;

            // 2) Drive + optional aim assist overlay
            driveFieldCentricWithOptionalAimAssist();

            telemetry.update();
        }

        limelight.stop();
    }

    // =========================================================
    // HARDWARE INIT
    // =========================================================
    private void initHardware() {
        // TODO: match these names to the RC config
        mFL = hardwareMap.get(DcMotor.class, "leftFront");
        mFR = hardwareMap.get(DcMotor.class, "rightFront");
        mBL = hardwareMap.get(DcMotor.class, "leftBack");
        mBR = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        // TODO: these directions are robot-specific (must verify)
        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        // TODO: IMU hub mounting (must match their hub orientation)
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Limelight
        // TODO: match this name to the config, and pipeline index to AprilTag pipeline
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(0);  // AprilTag pipeline
        limelight.start();
    }

    // =========================================================
    // DRIVE + AIM ASSIST
    // - Manual field-centric drive is always active
    // - Hold LEFT TRIGGER to add turn correction that centers the goal tag
    // =========================================================
    private void driveFieldCentricWithOptionalAimAssist() {

        // Field-centric drive from IMU
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double yawDeltaDeg = yawDeg - initYawDeg;

        double x = gamepad1.left_stick_x;      // strafe
        double y = -gamepad1.left_stick_y;     // forward (invert stick)
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

                // Deadband: ignore tiny errors to reduce “wiggle”
                if (Math.abs(txDeg) < TX_DEADBAND_DEG) txDeg = 0.0;

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

    // Utility
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
