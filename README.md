# FTC Decode (2025-2026)

- [This offical repository contains the public FTC SDK for the DECODE (2025-2026) competition season](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- [Game specific resources (official)](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)
   - FTC Decode Field CS: in the picture below. ![FTC Decode Field Map](images/decode-field_CS.png)
   - Robot CS: X+ --> Forward, Y+ --> Left, Z+ --> Up
   - Limelight CS: X+ --> camera right, Y+ --> Down, Z+ --> Out of camera

# Control Scheme Diagram (LotusTeleOPV33.java)

```
GAMEPAD 1 (Driver)                          GAMEPAD 2 (Operator)
──────────────────────────                  ──────────────────────────
LEFT STICK (Field-centric drive)            Y button
  ├─ Y-axis: forward/back                   └─ Toggle Flywheel (mFW)
  └─ X-axis: strafe                          (on/off each press)
RIGHT STICK X
  └─ Turn / rotate robot                    Right Bumper
                                           └─ Toggle Intake (sI)
Right Bumper
  └─ Toggle Ramp Wheel 1 (sRW1)              A button
   (on/off each press)                       └─ Toggle Ramp Wheel 2 (sRW2)
                                              (forward on/off)

                                           X button
                                           └─ Reverse Ramp Wheels:
                                              sRW2 and sRW1 at -0.25
                                              (toggle on/off)
```
# Useful links
## Goal
- [How To Shoot from Anywhere on the Field in Java FTC- Gyrobotic Droids 23435:graphing shooter method](https://www.youtube.com/watch?v=Mr2cLjyajnM)
- [How To Shoot from Anywhere with Velocity Compensation! Decode FTC](https://www.youtube.com/watch?v=oSVNTER_37A)
  - [Document that explains the theory behind our calculations](https://drive.google.com/file/d/1sEjsOUD9F9Mvefk5B-aR7Z1-Q-XwhLb8/view)

## Odometry
- [The Ultimate Guide to FTC Deadwheel Odometry](https://www.youtube.com/watch?v=RoDBoqOMc5A);
- [Learning Odometry for FTC - Part 1](https://www.youtube.com/watch?v=zsgJBK6KkV4)
- [Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry), SKU:3110-0002-0001](https://www.gobilda.com/pinpoint-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/?srsltid=AfmBOoqRlf2vj35s21_UKKVzE6fp11Ddf0jh33IJdZzZHtyQ9Qy_cwoa)
   - [3110-0002-0001 Odometry Computer User Guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf?srsltid=AfmBOoqx614LoFcLeFapvBPtQ76ab0gezAOg2nCGOXK7N6EGaJv98jm8)

## PID control or PIDF control
- [PID control](https://deep-blue-training.readthedocs.io/en/latest/section-7/feedback-control/)
- [This Trick Makes Your Flywheel 30x Faster (PIDF Tuning)](https://www.youtube.com/watch?v=aPNCpZzCTKg)
  

## Others:  
- [Template with the help of Claude Code to setup a team webpage hosted on GitHub](https://github.com/braineatingmachines/pitcrew)
  - [Example webpage--pitcrew](http://braineatingmachines.com/pitcrew/)


