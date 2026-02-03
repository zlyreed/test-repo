
// This OpMode will:
// Configure Pinpoint every time (required because settings reset on power cycle)
// Let you press buttons to reset pose + IMU and recalibrate IMU
// Print X, Y, heading (both normalized and unnormalized)
// Show encoder raw counts and device status to catch wiring issues

//Before running: In the Robot Controller configuration, add the Pinpoint as an I2C device and name it (example: "pinpoint").
/*On the Driver Station phone / Driver Hub
    This is the most common way teams do it.
   1) Open the FTC Driver Station app
   2) Tap the three dots (⋮) in the upper-right corner
   3) Tap Configure Robot
   4) Tap New (or Edit an existing config)
   5) Choose REV Control Hub (or Expansion Hub if applicable)
    
    You are now in the Robot Controller configuration screen.*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Odometry Test (GoBildaPinpointDriver)", group = "Debug")
public class PinpointOdometryTest_GoBildaDriver extends LinearOpMode {

    // -----------------------------
    // CHANGE THESE FOR YOUR ROBOT
    // -----------------------------
    private static final String PINPOINT_NAME = "pinpoint";

    // Pod type (pick the one you actually have)
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
            // GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

    // Offsets are in millimeters (mm) per goBILDA user guide.  :contentReference[oaicite:4]{index=4}
    // X pod = "forward pod" plugged into X port.
    // xOffset = how far LEFT(+)/RIGHT(-) the X pod is from your tracking point (usually robot center). :contentReference[oaicite:5]{index=5}
    // yOffset = how far FORWARD(+)/BACK(-) the Y pod is from your tracking point. :contentReference[oaicite:6]{index=6}
    private static final double X_POD_XOFFSET_MM = 0.0;   // left +, right -
    private static final double Y_POD_YOFFSET_MM = 0.0;   // forward +, back -

    // Encoder directions: X should increase when robot moves forward; Y should increase when robot moves left. :contentReference[oaicite:7]{index=7}
    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver(hardwareMap, PINPOINT_NAME);

        // -----------------------------
        // REQUIRED CONFIG (send every init; config is lost on power cycle) :contentReference[oaicite:8]{index=8}
        // -----------------------------
        pinpoint.setEncoderResolution(POD_TYPE);                               // ticks/mm preset for goBILDA pods :contentReference[oaicite:9]{index=9}
        pinpoint.setEncoderDirections(X_DIR, Y_DIR);                           // make X=forward+, Y=left+ :contentReference[oaicite:10]{index=10}
        pinpoint.setOffsets(X_POD_XOFFSET_MM, Y_POD_YOFFSET_MM);               // pod offsets in mm :contentReference[oaicite:11]{index=11}

        // Do one update so telemetry has real values before Start
        pinpoint.update();                                                     // data updates only when update() is called :contentReference[oaicite:12]{index=12}

        telemetry.addLine("Pinpoint Odometry Test Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = reset position + IMU (recommended at match start)");
        telemetry.addLine("  B = recalibrate IMU only (robot must be still)");
        telemetry.addLine("  X = toggle units (mm/in)");
        telemetry.update();

        boolean useInches = true;
        boolean lastX = false;

        waitForStart();

        // Recommended: reset pos + IMU once at the start when robot is stationary :contentReference[oaicite:13]{index=13}
        pinpoint.resetPosAndIMU();

        while (opModeIsActive()) {

            // Update sensor values
            pinpoint.update();

            // Toggle units with X
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastX) useInches = !useInches;
            lastX = xPressed;

            // Buttons
            if (gamepad1.a) {
                pinpoint.resetPosAndIMU(); // resets pose to 0,0,0 and recalibrates IMU :contentReference[oaicite:14]{index=14}
            }
            if (gamepad1.b) {
                pinpoint.recalibrateIMU(); // recalibrates IMU only (robot MUST be stationary) :contentReference[oaicite:15]{index=15}
            }

            // Device status
            GoBildaPinpointDriver.deviceStatus status = pinpoint.getDeviceStatus(); :contentReference[oaicite:16]{index=16}

            // Pose2D (normalized heading) :contentReference[oaicite:17]{index=17}
            Pose2D pose = pinpoint.getPosition();

            DistanceUnit du = useInches ? DistanceUnit.INCH : DistanceUnit.MM;

            double x = pose.getX(du);
            double y = pose.getY(du);
            double headingNormDeg = pose.getHeading(AngleUnit.DEGREES); // normalized -180..180 :contentReference[oaicite:18]{index=18}

            // Unnormalized heading directly from driver (radians) :contentReference[oaicite:19]{index=19}
            double headingUnnormDeg = Math.toDegrees(pinpoint.getHeading());

            // Raw encoders (useful for wiring/debug). Not affected by direction/scale. :contentReference[oaicite:20]{index=20}
            int encX = pinpoint.getEncoderX();
            int encY = pinpoint.getEncoderY();

            telemetry.addData("Status", status);
            telemetry.addData("Units", useInches ? "INCH" : "MM");

            telemetry.addData("X (forward)", "%.2f %s", x, useInches ? "in" : "mm");
            telemetry.addData("Y (left)", "%.2f %s", y, useInches ? "in" : "mm");

            telemetry.addData("Heading (normalized)", "%.1f deg", headingNormDeg);
            telemetry.addData("Heading (unnormalized)", "%.1f deg", headingUnnormDeg);

            telemetry.addData("EncX raw", encX);
            telemetry.addData("EncY raw", encY);

            telemetry.addLine("--- Expected ---");
            telemetry.addLine("Push forward  -> X increases");
            telemetry.addLine("Push left     -> Y increases");
            telemetry.addLine("Spin CCW      -> heading increases");

            telemetry.update();
        }
    }
}
