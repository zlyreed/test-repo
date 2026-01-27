

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

