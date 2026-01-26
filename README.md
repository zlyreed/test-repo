# test-FTC code
Work with Codex (ChatGPT) to write code for FTC Decode competition

# Control Scheme Diagram

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


