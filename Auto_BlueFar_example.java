package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Demo_BlueGoal_Column1_Intake3_Shoot3", group = "Autonomous")
public class Demo_BlueGoal_Column1_Intake3_Shoot3 extends LinearOpMode {

    // ----------------------------
    // TUNE THESE ON YOUR FIELD (inches + degrees)
    // ----------------------------
    public static double START_X = -52, START_Y = -60, START_H_DEG = 0;

    // "First column" pickup poses (3 stops)
    public static double BALL1_X = -24, BALL1_Y = -24, BALL1_H_DEG = 90;
    public static double BALL2_X = -24, BALL2_Y = -32, BALL2_H_DEG = 90;
    public static double BALL3_X = -24, BALL3_Y = -40, BALL3_H_DEG = 90;

    // Far launch/shoot pose (somewhere in the Launch Zone with line-of-sight to Blue goal)
    public static double SHOOT_X = -36, SHOOT_Y = -20, SHOOT_H_DEG = 45;

    // ----------------------------
    // TIMINGS (ms) - tune these
    // ----------------------------
    public static long INTAKE_MS_PER_BALL = 550;     // how long to run intake at each pickup
    public static long INDEX_MS = 250;               // brief ramp/index help
    public static long FLYWHEEL_SPOOL_MS = 1200;     // spool time before feeding (tune!)
    public static long FEED_MS = 450;                // feeding pulse per shot (tune!)

    // ----------------------------
    // LIMELIGHT AIMING - tune these
    // ----------------------------
    public static int BLUE_GOAL_TAG_ID = 20;         // common reference for Blue goal tag :contentReference[oaicite:3]{index=3}
    public static int LL_PIPELINE_BLUE_GOAL = 0;     // set your Limelight pipeline 0 = Blue goal-only
    public static double AIM_TOL_DEG = 1.0;          // stop aiming when |tx| < this
    public static double AIM_KP = 0.02;              // turn power per deg of tx
    public static double AIM_MAX_TURN = 0.6;
    public static double AIM_TIMEOUT_S = 1.5;

    // ----------------------------
    // Hardware names must match Robot Config
    // ----------------------------
    private CRServo sI;      // intake
    private CRServo sRW1;    // ramp wheel 1 (index)
    private CRServo sRW2;    // ramp wheel 2 (feed to shooter)
    private DcMotorEx mFW;   // flywheel

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // ---- Mechanisms ----
        sI   = hardwareMap.get(CRServo.class, "sI");
        sRW1 = hardwareMap.get(CRServo.class, "sRW1");
        sRW2 = hardwareMap.get(CRServo.class, "sRW2");
        mFW  = hardwareMap.get(DcMotorEx.class, "mFW");

        // (Optional) set directions if needed
        // sI.setDirection(CRServo.Direction.REVERSE);
        // sRW1.setDirection(CRServo.Direction.REVERSE);
        // sRW2.setDirection(CRServo.Direction.REVERSE);

        // ---- Limelight ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(LL_PIPELINE_BLUE_GOAL);

        // ---- Road Runner drive ----
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_H_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build key poses
        Pose2d ball1Pose = new Pose2d(BALL1_X, BALL1_Y, Math.toRadians(BALL1_H_DEG));
        Pose2d ball2Pose = new Pose2d(BALL2_X, BALL2_Y, Math.toRadians(BALL2_H_DEG));
        Pose2d ball3Pose = new Pose2d(BALL3_X, BALL3_Y, Math.toRadians(BALL3_H_DEG));
        Pose2d shootPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_H_DEG));

        waitForStart();
        if (isStopRequested()) return;

        // ----------------------------
        // 1) Drive to Ball #1 and intake
        // ----------------------------
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(new Vector2d(ball1Pose.position.x, ball1Pose.position.y), ball1Pose.heading)
                        .build()
        );
        intakeOneBall();

        // ----------------------------
        // 2) Drive to Ball #2 and intake
        // ----------------------------
        Actions.runBlocking(
                drive.actionBuilder(ball1Pose)
                        .strafeToLinearHeading(new Vector2d(ball2Pose.position.x, ball2Pose.position.y), ball2Pose.heading)
                        .build()
        );
        intakeOneBall();

        // ----------------------------
        // 3) Drive to Ball #3 and intake
        // ----------------------------
        Actions.runBlocking(
                drive.actionBuilder(ball2Pose)
                        .strafeToLinearHeading(new Vector2d(ball3Pose.position.x, ball3Pose.position.y), ball3Pose.heading)
                        .build()
        );
        intakeOneBall();

        // ----------------------------
        // 4) Drive to far shoot pose
        // ----------------------------
        Actions.runBlocking(
                drive.actionBuilder(ball3Pose)
                        .strafeToLinearHeading(new Vector2d(shootPose.position.x, shootPose.position.y), shootPose.heading)
                        .build()
        );

        // ----------------------------
        // 5) Aim to Blue Goal AprilTag with Limelight tx
        // ----------------------------
        aimWithLimelightTx(drive);

        // ----------------------------
        // 6) Shoot 3
        // ----------------------------
        shootThree();
    }

    private void intakeOneBall() {
        // intake + a little indexing help
        sI.setPower(1.0);
        sleep(INTAKE_MS_PER_BALL);

        sRW1.setPower(1.0);
        sleep(INDEX_MS);
        sRW1.setPower(0.0);

        sI.setPower(0.0);
    }

    private void shootThree() {
        // spool
        mFW.setPower(1.0);
        sleep(FLYWHEEL_SPOOL_MS);

        // 3 feed pulses
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            sRW2.setPower(1.0);
            sleep(FEED_MS);
            sRW2.setPower(0.0);
            sleep(150); // small settle gap (tune)
        }

        // stop
        mFW.setPower(0.0);
        sRW2.setPower(0.0);
        sRW1.setPower(0.0);
        sI.setPower(0.0);
    }

    private void aimWithLimelightTx(MecanumDrive drive) {
        // Simple “visual servo”: turn until Limelight tx ~ 0 deg.
        // Assumes your pipeline is filtering to the BLUE GOAL tag so tx refers to that target. :contentReference[oaicite:4]{index=4}

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.seconds() < AIM_TIMEOUT_S) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // If your pipeline is NOT tag-filtered, you can also check fiducial IDs here.
                // For simplicity, we assume the pipeline only returns the target tag.
                double tx = result.getTx(); // degrees :contentReference[oaicite:5]{index=5}

                if (Math.abs(tx) < AIM_TOL_DEG) break;

                double turn = clamp(AIM_KP * tx, -AIM_MAX_TURN, AIM_MAX_TURN);

                // Turn in place: vx=0, vy=0, omega=turn
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn));
                drive.updatePoseEstimate(); // keep RR localizer updating
            } else {
                // No target: stop turning and exit early
                break;
            }
        }

        // stop motion
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        drive.updatePoseEstimate();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
