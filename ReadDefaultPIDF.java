@TeleOp(name="Read Default PIDF", group="Test")
public class ReadDefaultPIDF extends OpMode {
    DcMotorEx mFW;

    @Override public void init() {
        mFW = hardwareMap.get(DcMotorEx.class, "mFW");
        mFW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients c = mFW.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("RUN_USING_ENCODER PIDF:");
        telemetry.addData("kP", c.p);
        telemetry.addData("kI", c.i);
        telemetry.addData("kD", c.d);
        telemetry.addData("kF", c.f);
        telemetry.update();
    }

    @Override public void loop() { }
}
