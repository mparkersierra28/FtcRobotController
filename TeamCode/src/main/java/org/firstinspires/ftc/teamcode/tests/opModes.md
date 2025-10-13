# FTC OpMode Lifecycle Guide

FTC has two main OpMode types: **Iterative** (`OpMode`) and **Linear** (`LinearOpMode`).  
Each has different lifecycle methods you can use.

---

## 📌 Iterative OpMode (`extends OpMode`)

Iterative OpModes run in phases with repeated calls to certain methods.

### `init()`
- Called once when the driver presses **INIT** on the Driver Station.
- Use it to:
    - Initialize your `RobotHardware`
    - Reset sensors
    - Set motor directions / modes
- **Does not loop.**

---

### `init_loop()`
- Runs repeatedly **after INIT is pressed but before START**.
- Good for:
    - Calibrating sensors (IMU, odometry, etc.)
    - Updating telemetry while waiting
    - Letting drivers confirm the robot is ready

---

### `start()`
- Called once **when START is pressed**.
- Use it to:
    - Reset timers
    - Zero encoders or variables if needed
    - Begin actions that should only happen after the match begins

---

### `loop()`
- Runs repeatedly **after START until STOP**.
- This is where the main driver-control or autonomous logic lives.
- Usually:
    - Read gamepad inputs
    - Set motor powers
    - Run autonomous state machines
    - Update telemetry

---

### `stop()`
- Called once **when STOP is pressed**.
- Use it to:
    - Power off motors/servos
    - Save logs or cleanup

---

## 📌 Linear OpMode (`extends LinearOpMode`)

Linear OpModes use one long method `runOpMode()` instead of lifecycle functions.

### `runOpMode()`
- Runs once when INIT is pressed.
- Typically:
    1. Initialize hardware
    2. Show telemetry while waiting
    3. Call `waitForStart()` to pause until START
    4. After START, your code runs sequentially
    5. Ends when method finishes or STOP is pressed

---

### Common Linear OpMode functions:

- **`waitForStart()`**  
  Blocks until START is pressed. Use before starting movement.

- **`opModeIsActive()`**  
  Returns `true` while OpMode is running and not stopped.  
  Use in loops to prevent the robot from running after STOP.

- **`sleep(ms)`**  
  Pauses execution for a set number of milliseconds.

---

## ✅ Best Practices

- Keep **all hardware initialization** in `init()` (Iterative) or before `waitForStart()` (Linear).
- Use **`init_loop()`** for calibration or updating telemetry while waiting.
- Put main driving/autonomous logic in **`loop()`** (Iterative) or after `waitForStart()` (Linear).
- Always check **`opModeIsActive()`** in Linear loops so your robot stops when the match ends.
- Use **`stop()`** to ensure motors are powered down safely.

---

This reference lets you quickly see **what each function does and when to use it**.
