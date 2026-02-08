/* Check if this is Pinpoint Odometry (SKU:3110-0002-0001 )
What this code does:
- Initializes the Pinpoint
- Configures pod type, directions, and offsets
- Resets pose at start
Continuously prints:
X (forward)
Y (left)
Heading (yaw, from IMU)
--------
Hardware assumptions:
Pinpoint added in Robot Controller configuration as an I2C device
Device name: "pinpoint"
2-dead-wheel setup (X pod + Y pod)
Pinpoint mounted flat
  */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "PinpointOdometry_example", group = "Robot")
public class PinpointOdometry_example extends LinearOpMode {

    private static final String PINPOINT_NAME = "pinpoint";

    @Override
    public void runOpMode() {

        // Initialize Pinpoint
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // -----------------------------
        // REQUIRED CONFIGURATION
        // -----------------------------

        // Select the odometry pod type you are using
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Encoder directions:
        // X pod = forward/back
        // Y pod = left/right
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Offsets from robot center (mm)
        // Replace with your measured values
        pinpoint.setOffsets(-114.3, 38.1, DistanceUnit.MM);
// X pod left(+)/right(-)
// Y pod forward(+)/back(-)
        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Keep robot still during INIT.");
        telemetry.update();

        waitForStart();

        // Reset pose and IMU at start
        pinpoint.resetPosAndIMU();

        while (opModeIsActive()) {

            // Update Pinpoint (VERY IMPORTANT)
            pinpoint.update();

            // Get robot pose
            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            telemetry.addData("X (forward)", "%.2f in", x);
            telemetry.addData("Y (left)", "%.2f in", y);
            telemetry.addData("Heading", "%.1f deg", heading);

            telemetry.addLine("----------------------");
            telemetry.addLine("Expected:");
            telemetry.addLine("Forward push → X increases");
            telemetry.addLine("Left push → Y increases");
            telemetry.addLine("CCW rotate → heading increases");

            telemetry.update();
        }
    }
}
