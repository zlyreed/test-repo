package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Robot: LimelightTeleOp", group = "Robot")
public class LimelightTeleOp extends LinearOpMode {


    public DcMotor mFL = null;

    public DcMotor mFR = null;

    public DcMotor mBL = null;

    public DcMotor mBR = null;

    public DcMotorEx oL = null;


    public DcMotor levitateL = null;

    public DcMotor levitateR = null;

    public DcMotor intake = null;

    public Servo pushUp = null;

    public Servo unYeet = null;

    public Servo shiny = null;

    Limelight3A limelight;
   // double flywheelVelocity = 50.0;

    @Override
    public void runOpMode() {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double drive;
        double strafeLeft;
        double strafeRight;
        double strafe;
        double turn;
        double maxBack;
        double maxFront;
        double intakeCounter = 0;
        double rS1Position = 0.3;
        double rS2Position = 0.32;




        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        levitateL = hardwareMap.get(DcMotor.class, "levitateL");
        levitateR = hardwareMap.get(DcMotor.class, "levitateR");


        shiny = hardwareMap.get(Servo.class,"shiny");



        oL = hardwareMap.get(DcMotorEx.class, "oL");

        intake = hardwareMap.get(DcMotor.class, "intake");
        pushUp = hardwareMap.get(Servo.class, "pushUp");
        unYeet = hardwareMap.get(Servo.class, "unYeet");



        levitateL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        levitateR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        levitateL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        levitateR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        mFL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);

        levitateR.setDirection(DcMotor.Direction.FORWARD);
        levitateL.setDirection(DcMotor.Direction.REVERSE);

        levitateL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        levitateR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        oL.setDirection(DcMotor.Direction.FORWARD);






        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);



        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
// DRIVETRAIN
            drive = - gamepad1.left_stick_y;
            double turnManual = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            // Aim assist overlay (only when held)
            double turnAssist = 0.0;
            boolean aimAssistEnabled = gamepad1.left_trigger > 0.2;
            
            if (aimAssistEnabled) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    // Find the tag you care about (example: pick one ID)
                    int desiredId = 20; // TODO change (Blue goal)
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            
                    if (fiducials != null) {
                        for (LLResultTypes.FiducialResult f : fiducials) {
                            if (f.getFiducialId() == desiredId) {
                                double tx = f.getTargetXDegrees();
                                // optional deadband
                                if (Math.abs(tx) < 1.0) tx = 0;
                                turnAssist = clamp(tx * 0.02, -0.35, 0.35);
                                break;
                            }
                        }
                    }
                }
            }
            
            turn = turnManual + turnAssist;


            frontLeft = drive + strafe + turn;
            frontRight = drive - strafe - turn;
            backLeft   = drive - strafe + turn;
            backRight  = drive + strafe - turn;

            maxBack = Math.max(Math.abs(backLeft), Math.abs(backRight));
            if (maxBack > 1.0)
            {
                backLeft /= maxBack;
                backRight /= maxBack;
            }

            maxFront = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
            if (maxFront > 1.0)
            {
                frontLeft /= maxFront;
                frontRight /= maxFront;
            }

            mFL.setPower(frontLeft);
            mFR.setPower(frontRight);
            mBL.setPower(backLeft);
            mBR.setPower(backRight);

            //OUTTAKE
            if (gamepad2.left_bumper)
            {

                oL.setPower(1);

            }

            else if (gamepad1.right_bumper) {
                oL.setPower(-0.25);
            }

            else {
                oL.setPower(0);
            }

            //UNYEET
            if (gamepad1.left_bumper) {
                unYeet.setPosition(-22);
            }
//            else {
//                unYeet.setPosition(-0.3);
//            }


            //LIFT
            if (gamepad1.dpad_up) {
                levitateR.setPower(-1);
                levitateL.setPower(-1);
            }

            else if (gamepad1.dpad_down) {
                levitateL.setPower(1);
                levitateR.setPower(1);
            }

            else {
                levitateL.setPower(0);
                levitateR.setPower(0);
            }



            //INTAKE
            if (gamepad2.right_bumper)
            {
                intake.setPower(1);
            }

            else if (gamepad1.right_bumper)
            {
                intake.setPower(-0.25);
            }
            else {
                intake.setPower(0);
            }

            //PUSH UP
            if (gamepad2.y)
            {
                pushUp.setPosition(0.2);
            }
            else if (gamepad1.right_bumper)
            {
                pushUp.setPosition(0.7);
            }
            else {
                pushUp.setPosition(0.5);
            }

            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                if (result.isValid()) {
//                }
//            }

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("bot location", "("+ x +" , "+ y +")");
            }

//            if ((botpose.getPosition().x != 0) &&  (botpose.getPosition().y != 0)){
//                shiny.setPosition(0.444);
//                telemetry.addLine("Has position");
//            }
//            else {
//                shiny.setPosition(0.277);
//                telemetry.addLine("Doesn't have position");
//            }
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                double distance = result.getBotposeAvgDist();
                telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
            }

            if ((1.15 <= result.getBotposeAvgDist()) && (result.getBotposeAvgDist() <= 1.25)) {
                shiny.setPosition(0.45);
                telemetry.addLine("Has position");
            }
            else {
                shiny.setPosition(0.3);
                telemetry.addLine("Doesn't have position");

            }







//            telemetry.addData("tx", result.getTx());
//            telemetry.addData("ty", result.getTy());
//            telemetry.addData("Botpose", botpose.toString());
            telemetry.addData("Velocity oL", "oL: %f", oL.getVelocity());

            telemetry.addData("Position oL", oL.getCurrentPosition());



            telemetry.addData("front left power", "%.2f", frontLeft);
            telemetry.addData("front right power","%.2f", frontRight);
            telemetry.addData("back left power","%.2f", backLeft);
            telemetry.addData("back right power","%.2f", backRight);
            telemetry.addData("intake counter","%.2f", intakeCounter);
            telemetry.addData("mFL.getPower", "%.2f", mFL.getPower());

            telemetry.addData("pushUp position", "%.2f", pushUp.getPosition());
            telemetry.update();
        }




    }


}



