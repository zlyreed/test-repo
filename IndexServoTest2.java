/* Here is the proof of concept code Jaswanth put together for the 5-Turn servo. It is pretty clean and straight forward. 

you probably already have this info, but I wanted to share...
side note - I don't know a ton about servos of servo-features and did a little digging around (Google - lol). The servo creep could be from weight against the servo causing it to slip.  (google solutions) add a second servo to help with the load - use springs or rubber bands to help lighten the load. 

outside of that I have three other equipment options - you are more than welcome to try.
1 - gobilda power injector - will increase the power to the servo so it will run over the 5v the Rev hub pushes out
2 - y-cable to run code to two servos via one cable - if you go the 2nd servo route - this should keep them in sync better than managing them as two separate items. I think they would be seen as one programmable object.
3 - signal boosting servo cable - might help with a cleaner signal (esp used with the y-cable)

^^ just a brainstorming dump - your group knows your robot and team better than I do ^^
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IndexServoTest2 extends OpMode {

    private Servo indexer;

    // ===== SERVO STATE =====
    private double currentPosition = 0.085;

    // ===== OFFSETS =====
    private final double INTAKE_POSITION = 0.085;
    private final double SHOOT_POSITION  = 0.275;
    private final double A_INCREMENT     = 0.075;

    private final double MIN_POS = 0.0;
    private final double MAX_POS = 1.0;

    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastA = false;

    @Override
    public void init() {
        indexer = hardwareMap.get(Servo.class, "servo");
        indexer.setPosition(currentPosition);
    }

    @Override
    public void loop() {

        boolean xPressed = gamepad1.x && !lastX;
        boolean bPressed = gamepad1.b && !lastB;
        boolean aPressed = gamepad1.a && !lastA;

        // X Intake 
        if (xPressed) {
            currentPosition = INTAKE_POSITION;
        }

        // B Shoot
        if (bPressed) {
            currentPosition = SHOOT_POSITION;
        }

        // A Advance
        if (aPressed) {
            currentPosition += A_INCREMENT;
        }

        currentPosition = Math.max(MIN_POS, Math.min(MAX_POS, currentPosition));

        indexer.setPosition(currentPosition);

        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastA = gamepad1.a;

        telemetry.addData("Servo Position", currentPosition);
        telemetry.update();
    }
}
