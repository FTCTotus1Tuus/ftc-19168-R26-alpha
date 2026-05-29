# Robot Architecture Guide
*Last updated: May 29, 2026 — revisit when new subsystems, libraries, or season game elements are added.*

This guide explains how to build and extend the FTC 19168 robot codebase so it stays readable, maintainable, and easy to grow throughout a season. It is written for both junior developers and AI coding agents.

**Goal:** a barebones but complete robot skeleton where:
- the robot boots and drives reliably from day one
- hardware is mapped in exactly one place
- OpModes stay thin (they orchestrate, not own)
- subsystems own their own behavior
- constants live in dedicated config classes
- TeleOp and Autonomous share a common base structure

---

## 1. Core Design Principles

1. **OpModes orchestrate. Subsystems behave.**
   - OpModes read input and call subsystem methods.
   - Hardware mapping, device wiring, and behavior logic belong in lower layers.

2. **Separate hardware, subsystems, services, and config.**
   - `team/hardware/` — device mapping and safe startup defaults
   - `team/subsystems/` — one class per mechanism or behavior area
   - `team/services/` — follower, localization, vision, preferences
   - `team/config/` — all tunable constants

3. **Use intent methods. Never expose internal state.**
   - Good: `drive.setTeleOpDrive(x, y, rx)` 
   - Bad: writing directly to motor fields from an OpMode or another subsystem

4. **One source of truth per constant.**
   - Drive scaling → `DriveConfig`
   - Vision timeouts → `VisionConfig`
   - Auto paths and poses → `AutoConfig`

5. **The run loop is always deterministic:** read input → update subsystems → write outputs → telemetry.

6. **Prefer composition over deep inheritance.**
   - `RobotOpMode` is a thin lifecycle shell (one init method, one stop method).
   - Real wiring happens in `RobotContainer`.

7. **Only use `*FSM` for actual finite state machines.**
   - An FSM has a private `State` enum and conditional transitions between states.
   - An OpMode base class is not an FSM. A simple toggle is not an FSM.

---

## 2. Package Layout

```
org.firstinspires.ftc.teamcode
└── team/
    ├── config/           — @Config tuning constants only; no behavior
    ├── hardware/         — RobotHardware.java; all hardwareMap access lives here
    ├── subsystems/       — one class per mechanism or behavior area
    ├── services/         — follower, localization, vision, preferences
    ├── core/             — RobotContainer (composition root)
    ├── opmodes/
    │   ├── RobotOpMode.java       — thin LinearOpMode base class
    │   ├── TeleOpMode.java        — driver-control OpMode
    │   └── auto/                  — autonomous OpModes
    └── testing/          — tuning and diagnostic OpModes (never deployed to competition)
```

---

## 3. Class Architecture

### 3.1 Inheritance Tree

```
LinearOpMode  (FTC SDK — never modified)
└── RobotOpMode                   team/opmodes/RobotOpMode.java
    ├── TeleOpMode                 team/opmodes/TeleOpMode.java
    └── AutonomousBase             team/opmodes/auto/AutonomousBase.java
        ├── RedLeftAuto
        ├── RedRightAuto
        ├── BlueLeftAuto
        └── BlueRightAuto
```

**Why this shape?** One shared base class means `initRobot()` and `stopRobot()` are written once. Autonomous variants share path-building helpers from `AutonomousBase`. TeleOp stays completely separate from auto logic.

### 3.2 Composition Tree

```
RobotOpMode
└── RobotContainer                 team/core/RobotContainer.java
    ├── RobotHardware              team/hardware/RobotHardware.java
    ├── DriveSubsystem             team/subsystems/DriveSubsystem.java
    ├── LocalizationService        team/services/LocalizationService.java
    ├── VisionService              team/services/VisionService.java
    ├── PreferencesService         team/services/PreferencesService.java
    └── [season subsystems]        team/subsystems/ — added each season
```

### 3.3 Class Responsibilities

#### `RobotOpMode`
- The team's base class for all OpModes — extends `LinearOpMode`
- Owns `RobotContainer` and calls `initRobot()` / `stopRobot()`
- Provides shared telemetry helpers
- **Should stay small and stable across seasons** — do not add season-specific logic here

#### `TeleOpMode`
- Reads `gamepad1` / `gamepad2` inputs
- Calls subsystem intent methods (`drive.setTeleOpDrive(...)`)
- Manages driver-facing mode switches (e.g., field-centric toggle)
- Contains no hardware references — everything goes through `RobotContainer`

#### `AutonomousBase`
- Provides shared path-building helpers for all autonomous OpModes
- Writes alliance color and final robot pose to `PreferencesService` on finish
- Concrete subclasses override `buildPath()` or equivalent to define the route

#### `RobotContainer`
- The single composition root — creates, initializes, and tears down all subsystems and services
- Called by `RobotOpMode.initRobot()` and `RobotOpMode.stopRobot()`
- Exposes subsystem references as public fields (e.g., `container.drive`, `container.vision`)

#### `RobotHardware`
- **All** `hardwareMap.get()` calls happen here — nowhere else
- Sets motor directions, zero-power behaviors, and encoder mode defaults
- Returns typed device references consumed by subsystems
- Contains no behavior logic — hardware initialization only

#### `DriveSubsystem`
- Wraps the Pedro Pathing `Follower`
- Exposes: `setTeleOpDrive(x, y, rx)`, `followPath(path)`, `getPose()`, `stop()`
- Hides Pedro Pathing API details from OpModes so swapping the drive library is localized

#### `LocalizationService`
- Wraps the GoBilda Pinpoint driver (`GoBildaPinpointDriver`)
- Seeds the Pedro Pathing follower pose on init from Pinpoint
- Provides a safe fallback to drive-encoder odometry if Pinpoint is unavailable
- **Must not block robot startup if Pinpoint is missing**

#### `VisionService`
- Owns the `VisionPortal` and `AprilTagProcessor` lifecycle
- Provides `getAprilTagDetections()` as immutable snapshots
- Closes the vision portal cleanly in `stop()`
- **Must not block robot startup if the camera is missing or fails**

#### `PreferencesService`
- Persists alliance color and final robot pose from autonomous to TeleOp
- Uses FTC `opMode.blackboard` (SDK 10.3+) or Android `SharedPreferences`
- Accessed by autonomous to write, and by TeleOp to read and restore pose

---

## 4. When to Use a Finite State Machine

Use an FSM when a subsystem has **multiple discrete states and conditional transitions** between them.

**Good FSM candidates:**
- A multi-step intake sequence (idle → extending → intaking → retracting → idle)
- An autonomous routine with sequential, conditional steps
- An elevator with named positions (FLOOR, TRANSFER, SCORING)

**Simple subsystems that do NOT need an FSM:**
- A claw servo with two positions → use `open()` / `close()` directly
- A drive subsystem → use direct power methods
- A one-shot action → use a boolean flag or a single method call

**FSM pattern:**
```java
public class ExampleFSM {

    private enum State { IDLE, RUNNING, DONE }
    private State state = State.IDLE;

    // Intent methods — the only public API
    public void start()         { state = State.RUNNING; }
    public void cancel()        { state = State.IDLE; }
    public boolean isDone()     { return state == State.DONE; }

    // Called once per loop from RobotContainer or directly from an OpMode
    public void update() {
        switch (state) {
            case IDLE:
                break;
            case RUNNING:
                // do work; transition when complete
                state = State.DONE;
                break;
            case DONE:
                break;
        }
    }
}
```

The `State` enum is **always private**. External code uses intent methods only — never sets `state` directly.

---

## 5. GoBilda Pinpoint Odometry

The GoBilda Pinpoint is a self-contained I2C odometry computer. It fuses two dead-wheel encoders with an onboard IMU to produce X/Y/heading estimates, removing the need to run odometry through the drive encoders.

**Hardware setup:**
- Connect to any Control Hub I2C port (default I2C address `0x31`)
- Add to `RobotHardwareNames`: `public static final String PINPOINT = "pinpoint";`
- Retrieve in `RobotHardware`: `hardwareMap.get(GoBildaPinpointDriver.class, RobotHardwareNames.PINPOINT)`
- The driver class ships with the FTC SDK (v10.3+) — no extra Gradle dependency needed

**Software flow:**
1. `LocalizationService` holds the `GoBildaPinpointDriver` reference
2. On `initialize()`: call `pinpoint.resetPosAndIMU()`, then seed the Pedro Pathing follower pose
3. Each loop: call `pinpoint.update()`, then `pinpoint.getPosition()` to read the current pose
4. Pedro Pathing is configured to use the Pinpoint via `FollowerBuilder.pinpointLocalizer(...)`

**Key API:**
```java
GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
pinpoint.resetPosAndIMU();   // call once during init
// each loop:
pinpoint.update();
Pose2D pose = pinpoint.getPosition();  // returns inches + radians
```

**Failure handling:** wrap Pinpoint init in a try/catch. Log the error and fall back to drive-encoder odometry. The robot must remain drivable.

---

## 6. Pedro Pathing Integration

Pedro Pathing (`com.pedropathing:ftc:2.1.2`, `maven { url = 'https://maven.brott.dev/' }`) provides smooth autonomous path following and field-centric TeleOp drive assistance.

- The follower is created via `PedroPathingConstants.createFollower(hardwareMap)` (already implemented)
- Motor names are auto-resolved at runtime by `resolveConfiguredMotorNames()` — handles mismatched config names gracefully
- **Runtime pose source:** always `follower.getPose()` — never read raw encoder values directly for pose
- **TeleOp drive:** `follower.setTeleOpDrive(y, x, rx, fieldCentric)`
- **All Pedro Pathing tuning values** live in `PedroPathingConstants`; run tuning OpModes from `team/testing/` before competition

---

## 7. Configuration Classes

Every tunable value belongs in a `*Config` class under `team/config/`. Annotate with `@Config` (from **FtcDashboard** — see §14) so that `public static` fields in the class become live-editable in the dashboard web UI without redeploying.

| Class | Owns |
|---|---|
| `DriveConfig` | drive scaling, field-centric toggle, max power |
| `OdometryConfig` | Pinpoint pod offsets, wheel diameter, ticks/revolution |
| `VisionConfig` | AprilTag IDs, camera exposure and gain, vision timeouts |
| `AutoConfig` | starting poses, parking poses, timing margins |

**Rules:**
- No magic numbers or string literals in OpModes or subsystems — always use a named constant
- No control flow or state in config classes — constants only
- One config class per concern — do not merge `DriveConfig` and `VisionConfig`
- All fields that should be dashboard-editable must be `public static` (not `final`)

---

## 8. Adding a New Subsystem

Follow this order every time. Each step should compile and leave the robot in a working state.

1. **Config** — `team/config/NewSubsystemConfig.java` (only if it has tunable values)
2. **Hardware** — add device constant to `RobotHardwareNames.java` and mapping to `RobotHardware.java`
3. **Subsystem** — `team/subsystems/NewSubsystem.java` with `initialize()`, `update()`, `stop()`
4. **Wire** — add the subsystem to `RobotContainer` (construct, init, update, stop)
5. **Use** — call from `TeleOpMode` or an autonomous OpMode via `robot.newSubsystem.method()`

Never wire a subsystem directly inside an OpMode.

**Example — adding a claw:**
- `team/config/ClawConfig.java` — servo positions for open/close
- `RobotHardware.java` — `hardwareMap.get(Servo.class, RobotHardwareNames.CLAW)`
- `team/subsystems/ClawSubsystem.java` — `open()`, `close()`, `initialize()`
- `RobotContainer` — creates and owns `ClawSubsystem`
- `TeleOpMode` — calls `robot.claw.open()` on a gamepad button press

---

## 9. Startup and Shutdown Flow

### Init sequence
```
runOpMode()
  → initRobot()
      → RobotContainer.initialize(hardwareMap, telemetry)
          → RobotHardware.initialize(hardwareMap)      // all hardwareMap.get() calls here
          → LocalizationService.initialize()           // Pinpoint reset + follower seed
          → DriveSubsystem.initialize()                // Pedro Pathing follower ready
          → VisionService.initialize()                 // VisionPortal open (fail-safe)
          → PreferencesService.initialize()            // restore saved pose if available
          → [season subsystems].initialize()
  → telemetry.addLine("Ready"); telemetry.update()
  → waitForStart()
```

### Stop sequence
```
opModeIsActive() returns false
  → stopRobot()
      → VisionService.stop()          // close VisionPortal
      → DriveSubsystem.stop()         // halt follower
      → [season subsystems].stop()
```

**Rule:** any `initialize()` that can fail due to missing hardware must be wrapped in a try/catch. Log the failure to telemetry and RobotLog, then continue. The robot must always be drivable after init, even if optional systems are down.

---

## 10. Code Templates

### `RobotOpMode` (base class)
```java
package org.firstinspires.ftc.teamcode.team.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.team.core.RobotContainer;

/**
 * RobotOpMode — thin base class for all team OpModes.
 * Owns the RobotContainer lifecycle. Subclasses implement runOpMode().
 */
public abstract class RobotOpMode extends LinearOpMode {

    protected RobotContainer robot;

    protected void initRobot() {
        robot = new RobotContainer(hardwareMap, telemetry);
        robot.initialize();
    }

    protected void stopRobot() {
        if (robot != null) {
            robot.stop();
        }
    }
}
```

### `TeleOpMode`
```java
@TeleOp(name = "TeleOp", group = "DriverControl")
public class TeleOpMode extends RobotOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        telemetry.addLine("Ready"); telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // 1. Read input
            double x  = gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // 2. Update subsystems
            robot.drive.setTeleOpDrive(x, y, rx);

            // 3. Telemetry
            telemetry.addData("Pose", robot.drive.getPose());
            telemetry.update();
            idle();
        }

        stopRobot();
    }
}
```

### Subsystem
```java
/**
 * DriveSubsystem — wraps the Pedro Pathing Follower.
 * Call setTeleOpDrive() each loop in TeleOp. Use followPath() in autonomous.
 * OpModes must not access motors directly — use this class's public API.
 */
public class DriveSubsystem {

    private final Follower follower;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = PedroPathingConstants.createFollower(hardwareMap);
    }

    public void initialize() {
        follower.startTeleopDrive(true);
    }

    public void setTeleOpDrive(double x, double y, double rx) {
        follower.setTeleOpDrive(y, x, rx, true);
        follower.update();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void stop() {
        follower.breakFollowing();
    }
}
```

### Generic class structure
```java
package org.firstinspires.ftc.teamcode.team.subsystems;

/**
 * ExampleSubsystem — one sentence: what does this class own?
 * Callers should use: [list the public intent methods].
 * Callers should NOT: [what to avoid accessing directly].
 */
public class ExampleSubsystem {

    // Private fields only

    public ExampleSubsystem(/* dependencies */) {
        // store dependencies; do not call hardwareMap here
    }

    public void initialize() {
        // safe startup; set device defaults
    }

    public void update() {
        // called each loop; advance internal state
    }

    public void stop() {
        // release resources or set safe output
    }
}
```

### Finite state machine
```java
public class ExampleFSM {

    private enum State { IDLE, ACTIVE, DONE }
    private State state = State.IDLE;

    // Public API — intent methods only
    public void activate()      { state = State.ACTIVE; }
    public void reset()         { state = State.IDLE; }
    public boolean isIdle()     { return state == State.IDLE; }
    public boolean isDone()     { return state == State.DONE; }

    public void update() {
        switch (state) {
            case IDLE:
                break;
            case ACTIVE:
                // run work; transition when complete
                state = State.DONE;
                break;
            case DONE:
                break;
        }
    }
}
```

---

## 11. First-Week Season Checklist

Build in this order. Do not move to the next step until the current one compiles and runs on the robot.

- [ ] `RobotHardwareNames` has constants for all installed devices (motors, Pinpoint, camera)
- [ ] `RobotHardware` maps all devices and sets safe startup defaults
- [ ] `LocalizationService` reads pose from Pinpoint and seeds the Pedro Pathing follower
- [ ] `DriveSubsystem` drives the robot in TeleOp (field-centric, gamepad1 sticks)
- [ ] `TeleOpMode` boots, drives, and shows live pose in telemetry
- [ ] `RobotContainer` composes hardware, drive, and localization; `RobotOpMode` boots cleanly
- [ ] `VisionService` opens (or fails silently) with AprilTag processor ready
- [ ] One autonomous OpMode follows a simple straight-line path and parks
- [ ] `PreferencesService` saves final pose after auto and restores it at TeleOp init
- [ ] All tunable values (`DriveConfig`, `OdometryConfig`, `AutoConfig`) exposed via FTC Dashboard

---

## 12. What NOT to Do

- Do not edit `FtcRobotController/` — it is the upstream SDK
- Do not call `hardwareMap.get()` outside `RobotHardware.java`
- Do not embed constants or magic numbers inside OpModes or subsystems
- Do not name a class `*FSM` unless it has a private `State` enum and conditional transitions
- Do not block robot startup when an optional system (camera, Pinpoint) fails to initialize
- Do not bypass `RobotContainer` to wire subsystems directly in an OpMode
- Do not add season-specific logic to `RobotOpMode` — it must remain stable across seasons
- Do not let a class grow until it is hard to explain in one sentence — split it instead

---

## 13. Decision Guide: Where Does New Code Belong?

When unsure, answer these questions in order:

1. Is it a physical device? → `RobotHardware`
2. Is it a tunable constant? → `team/config/*Config.java`
3. Is it a shared software service (follower, vision, preferences)? → `team/services/`
4. Is it behavior for one mechanism? → `team/subsystems/`
5. Is it orchestration (input + delegation)? → `team/opmodes/`

---

## 14. Native FTC SDK Patterns for Performance & Control

Rather than relying on third-party libraries that may become unmaintained, this team uses only the native FTC SDK plus **FtcDashboard** (for live tuning and telemetry). This keeps the codebase lightweight, stable, and future-proof.

---

### 14.1 FtcDashboard

**Maven:** `maven { url = "https://maven.brott.dev/" }`  
**Dependency:** `implementation 'com.acmerobotics.dashboard:dashboard:0.6.0'` (or latest v0.6.x)

FtcDashboard streams telemetry and lets you edit `public static` fields on any `@Config`-annotated class live from a browser at `http://192.168.43.1:8080/dash` while the OpMode is running. No restart needed.

**Usage pattern (config class):**
```java
import com.acmerobotics.dashboard.config.Config;

@Config  // exposes all public static fields to the dashboard
public class LiftConfig {
    public static double P = 0.005;
    public static double I = 0.0;
    public static double D = 0.0001;
    public static int POS_HIGH = 2500;
    public static int POS_TRANSFER = 1200;
    public static int POS_DOWN = 0;
}
```

**Rules:**
- Only `public static` (non-`final`) fields are editable from the dashboard.
- Never annotate a subsystem class with `@Config` — config belongs in `team/config/`.
- Dashboard telemetry uses `FtcDashboard.getInstance().getTelemetry()`; the driver-station telemetry object is separate.

---

### 14.2 Native Bulk Caching (RobotHardware)

The FTC SDK's `LynxModule.BulkCachingMode` dramatically improves loop speed by batching hardware reads into one I2C transaction per hub instead of many tiny reads scattered throughout subsystem `update()` methods.

**Setup (in `RobotHardware.initialize()`):**
```java
public void initialize(HardwareMap hardwareMap) {
    // Get all LynxModule (REV Control Hub / Expansion hub) references
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    // Enable manual bulk caching mode on all hubs
    for (LynxModule hub : allHubs) {
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // Now fetch all your devices (motors, servos, sensors)
    // ...hardwareMap.get() calls for all motors, servos, sensors...
}

public void clearBulkCache() {
    // Call this at the start of each OpMode loop
    for (LynxModule hub : allHubs) {
        hub.clearBulkCache();
    }
}
```

**Usage (in OpMode loop):**
```java
while (opModeIsActive()) {
    // 1. Clear bulk cache at START of loop so all subsequent reads get fresh data
    robot.hardware.clearBulkCache();

    // 2. Update all subsystems — they read motor encoders, gyro, etc. from cache
    robot.drive.update();
    robot.lift.update();
    robot.vision.update();

    // 3. Write outputs (motor powers, servo positions)
    // (no code here — subsystems already wrote them via setPower/setPosition)

    telemetry.update();
}
```

**Key rules:**
- Store the `allHubs` list in `RobotHardware` and expose a `clearBulkCache()` method.
- Call `clearBulkCache()` at the **start** of every OpMode loop before subsystem updates.
- All subsequent reads from motors/sensors within the same loop pass the cached data — safe and fast.
- Set `AUTO` mode only if you have very few devices and don't care about latency.

---

### 14.3 Native PIDF Control (Custom Calculator + @Config)

The FTC SDK provides `PIDFCoefficients` — a lightweight, standard container. Write a small (15–20 line) calculator class that reads from a `@Config` class and runs error-correction math each loop.

**Config class (team/config/LiftConfig.java):**
```java
import com.acmerobotics.dashboard.config.Config;

@Config
public class LiftConfig {
    public static double P = 0.005;
    public static double I = 0.0;
    public static double D = 0.0001;
    public static double F = 0.0;  // feedforward
    public static int POS_HIGH = 2500;
    public static int POS_TRANSFER = 1200;
}
```

**Calculator class (team/subsystems/LiftPIDFCalculator.java):**
```java
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.team.config.LiftConfig;

public class LiftPIDFCalculator {
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();

    public double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;
        double deltaTime = loopTimer.seconds();
        loopTimer.reset();

        // Integrate error over time (with anti-windup cap)
        integralSum += error * deltaTime;
        integralSum = Math.max(-100, Math.min(100, integralSum));

        // Derivative of error
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // PIDF output
        double power = (LiftConfig.P * error)
                     + (LiftConfig.I * integralSum)
                     + (LiftConfig.D * derivative)
                     + LiftConfig.F;  // constant feedforward

        return Math.max(-1, Math.min(1, power));
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        loopTimer.reset();
    }
}
```

**Usage (in `LiftSubsystem.update()`):**
```java
public class LiftSubsystem {

    private final DcMotorEx motor;
    private final LiftPIDFCalculator pidf = new LiftPIDFCalculator();
    private double targetPosition = 0;

    public LiftSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.LIFT_MOTOR);
    }

    public void update() {
        double power = pidf.calculate(targetPosition, motor.getCurrentPosition());
        motor.setPower(power);
    }

    public void setTargetPosition(double ticks) {
        targetPosition = ticks;
    }

    public void stop() {
        pidf.reset();
        motor.setPower(0);
    }
}
```

**Live tuning flow:**
1. Run your OpMode with this code.
2. Open `http://192.168.43.1:8080/dash` in a browser.
3. Edit `LiftConfig.P`, `.I`, `.D`, `.F` in real time.
4. Watch the lift respond instantly (because `calculate()` reads from the config class every frame).
5. Once tuned, copy the final values back into `LiftConfig.java` as defaults.

**Why this approach:**
- No external dependencies — 100% safe long-term.
- Teaches junior students how PIDF actually works (P gain × error + I × integral + D × derivative).
- Pairs perfectly with FtcDashboard for live tuning.
- Extremely predictable — no hidden behavior.

---

*See also: `AGENTS.md` at project root for a concise AI coding quick-reference.*
