# Control Scheme Improvement Suggestions

## 1) Add explicit debounce / edge detection for toggles
The current toggles rely on `*_WasPressed()` and counters. If button state changes are sampled rapidly, it can cause unintended toggles. Add explicit edge detection (previous vs. current state) or a short cooldown timer to ensure a single toggle per press.

## 2) Use boolean state flags instead of modulo counters
Counters with `% 2` checks work but can be confusing, especially when multiple buttons affect the same mechanism. Use explicit boolean state variables such as `flywheelOn`, `intakeOn`, `rw1On`, `rw2ForwardOn`, and `rw2ReverseOn` for clarity and safer logic.

## 3) Resolve ramp wheel 2 input conflicts explicitly
Both `A` and `X` affect ramp wheel 2 (and `X` also affects ramp wheel 1). Define explicit priority or mutual exclusivity (e.g., reverse overrides forward, or pressing one disables the other) to avoid ambiguous states.

## 4) Add ramp-up / ramp-down transitions for motors and servos
Instantly jumping from 0 to full power can stress hardware. Implement a simple ramp (e.g., step power by 0.1–0.2 per loop) when enabling or disabling the flywheel and intake/ramp servos.

## 5) Add safety interlocks between subsystems
If desired, enforce sequencing such as only enabling ramp wheels when the flywheel is running to reduce jams or misfeeds. This can be done with simple conditional checks before enabling actuators.
