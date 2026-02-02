# FTC Decode (2025-2026)
- Work with Codex (ChatGPT) to write code for FTC Decode competition
- [This offical repository contains the public FTC SDK for the DECODE (2025-2026) competition season](https://github.com/FIRST-Tech-Challenge/FtcRobotController)

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
  
## PID control or PIDF control
- [PID control](https://deep-blue-training.readthedocs.io/en/latest/section-7/feedback-control/)
- [This Trick Makes Your Flywheel 30x Faster (PIDF Tuning)](https://www.youtube.com/watch?v=aPNCpZzCTKg)
  

## Others:  
- [Template with the help of Claude Code to setup a team webpage hosted on GitHub](https://github.com/braineatingmachines/pitcrew)
  - [Example webpage--pitcrew](http://braineatingmachines.com/pitcrew/)


