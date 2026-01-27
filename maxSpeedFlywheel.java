// Max speed with using encoder (may not be the true max speed)
// Constants (change to your motor)
double TICKS_PER_REV = 28;  // example: goBILDA 5203;  SKU: 5203-2402-0001

// State variables
boolean flywheelOn = false;
double maxRPMObserved = 0.0;

@Override
public void runOpMode() {

    DcMotorEx mFW = hardwareMap.get(DcMotorEx.class, "mFW");
    mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    while (opModeIsActive()) {

        // Toggle flywheel ON/OFF with gamepad2 Y
        if (gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;
        }

        if (flywheelOn) {
            // 1️⃣ Full power
            mFW.setPower(1.0);

            // 2️⃣ Measure velocity (ticks/sec → RPM)
            double currentRPM =
                    mFW.getVelocity() * 60.0 / TICKS_PER_REV;

            // 3️⃣ Track maximum RPM reached
            if (currentRPM > maxRPMObserved) {
                maxRPMObserved = currentRPM;
            }

            telemetry.addData("Flywheel", "ON (setPower=1.0)");
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Max RPM Seen", "%.0f", maxRPMObserved);

        } else {
            mFW.setPower(0.0);
            telemetry.addData("Flywheel", "OFF");
        }

        telemetry.update();
    }
}

///////////////////////////////////////////////////////////////////////////////
// MaxSpeed without using Encoder
@TeleOp(name="Flywheel Max Speed Test")
public class FlywheelMaxSpeedTest extends LinearOpMode {

    // goBILDA 5203 motor encoder counts per MOTOR revolution
    static final double TICKS_PER_REV = 28.0;

    boolean flywheelOn = false;
    boolean lastY = false;

    double maxTicksPerSecObserved = 0.0;
    double maxRPMObserved = 0.0;

    @Override
    public void runOpMode() {

        DcMotorEx mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        mFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // Edge-detect Y to toggle ON/OFF
            boolean y = gamepad2.y;
            if (y && !lastY) {
                flywheelOn = !flywheelOn;

                // Optional: reset peak each time you turn it ON
                if (flywheelOn) {
                    maxTicksPerSecObserved = 0.0;
                    maxRPMObserved = 0.0;
                }
            }
            lastY = y;

            if (flywheelOn) {
                mFW.setPower(1.0);

                double ticksPerSec = mFW.getVelocity();               // encoder ticks/sec
                double rpm = ticksPerSec * 60.0 / TICKS_PER_REV;      // motor RPM

                if (ticksPerSec > maxTicksPerSecObserved) maxTicksPerSecObserved = ticksPerSec;
                if (rpm > maxRPMObserved) maxRPMObserved = rpm;

                telemetry.addData("Flywheel", "ON (power=1.0)");
                telemetry.addData("Current (ticks/s)", "%.0f", ticksPerSec);
                telemetry.addData("Peak (ticks/s)", "%.0f", maxTicksPerSecObserved);
                telemetry.addData("Current (motor RPM)", "%.0f", rpm);
                telemetry.addData("Peak (motor RPM)", "%.0f", maxRPMObserved);

            } else {
                mFW.setPower(0.0);
                telemetry.addData("Flywheel", "OFF");
                telemetry.addData("Peak (ticks/s)", "%.0f", maxTicksPerSecObserved);
                telemetry.addData("Peak (motor RPM)", "%.0f", maxRPMObserved);
            }

            telemetry.update();
        }
    }
}



/////////////////////////////////////////////////////////////
// to test TICKS_PER_REV
//Results interpretation
//If it shows ~28 → use 28
//If it shows ~112 → use 112
//If it shows ~537 → gearbox is involved (old value)
//This test is bulletproof and worth doing once.
    
mFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Slowly turn flywheel by hand exactly ONE revolution
// then read:
telemetry.addData("Encoder position", mFW.getCurrentPosition());

