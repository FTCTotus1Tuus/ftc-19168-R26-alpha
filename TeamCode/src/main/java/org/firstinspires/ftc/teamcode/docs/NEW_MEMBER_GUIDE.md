# New Member Guide — FTC 19168 Robot Code

**Brand new? Read [START_HERE.md](START_HERE.md) first** — it's 10 minutes and
assumes nothing. Come back here once it makes sense.

This doc is step two. It explains how our robot code is organized and how to
make your first change. It assumes you know a little Java (variables, `if`,
methods, classes) and nothing about FTC.

When you're comfortable here, read `ROBOT_ARCHITECTURE_GUIDE.md` — that's the
deep version with all the rules and rationale. This doc is the tour.

---

## 1. The one-paragraph version

Our robot is a **mecanum drive** (4 wheels that let it slide sideways) with an
**intake** (spinning rubber bands that suck balls in), a **webcam**, and an
**odometry sensor** that tracks where the robot is on the field. The code is
split so that *one class talks to the hardware*, *each mechanism gets its own
class*, and *the driver-facing programs (OpModes) just give orders*. Nobody
reaches around anybody else.

---

## 2. What an OpMode is

In FTC, the code you actually run on the robot is called an **OpMode**. Two kinds:

| Kind | Annotation | When it runs | How long |
|---|---|---|---|
| Autonomous | `@Autonomous` | first 30 seconds | robot drives itself |
| TeleOp | `@TeleOp` | rest of the match | humans drive with gamepads |

An OpMode has two phases:

```java
// everything before waitForStart() runs when you press INIT on the Driver Station
waitForStart();          // code pauses here until the driver presses PLAY
while (opModeIsActive()) {
    // this loop runs ~50 times per second until the match ends or STOP is pressed
}
```

**That loop is the single most important idea in FTC.** Your code does not
"drive forward 3 feet" as one instruction. It runs the loop body over and over,
and each pass it looks at the current situation and sets motor powers *for right
now*. Driving forward 3 feet means "keep setting forward power until the sensors
say we've gone 3 feet."

Our OpModes live in [`v1/opmodes/`](../v1/opmodes/). Start with
[TeleOpMode.java](../v1/opmodes/TeleOpMode.java) — it's the simplest complete one.

---

## 3. The layers (why there are so many folders)

Everything lives under `teamcode/v1/`. The `v1` means "version 1 of this
season's code" — if we ever want to rewrite from scratch we make `v2/` and keep
`v1/` working. Inside:

```
v1/
├── opmodes/      ← the programs you run. Read gamepads, give orders.
├── core/         ← RobotContainer: builds the whole robot, one object.
├── subsystems/   ← one class per mechanism (DriveSubsystem, IntakeSubsystem).
├── services/     ← helpers that aren't mechanisms (VisionService, odometry, saved settings).
├── hardware/     ← the ONLY place allowed to ask the Driver Station for a motor.
├── config/       ← plain numbers you tune. No logic, just constants.
└── testing/      ← tuning + diagnostic OpModes. Not used in matches.
```

The rule that makes this work:

> **An OpMode never touches a motor directly.** It calls a subsystem.
> **A subsystem never looks up hardware itself.** It asks `RobotHardware`.
> **Nobody hardcodes a number.** Numbers live in `config/`.

Why bother? Because when the intake motor gets rewired to a different port, you
change **one line** in [RobotHardwareNames.java](../v1/hardware/RobotHardwareNames.java)
instead of hunting through five files.

### `RobotContainer` — the robot in one object

[core/RobotContainer.java](../v1/core/RobotContainer.java) builds every subsystem
once and hands them out as public fields. Every OpMode extends
[RobotOpMode](../v1/opmodes/RobotOpMode.java), which gives you a field called
`robot`. So from any OpMode you can write:

```java
robot.drive.setTeleOpDrive(forward, strafe, turn);
robot.intake.start();
robot.vision.getBallTarget();
```

That's the whole API you need to know to start.

---

## 4. Our actual hardware

Seven devices total. Names on the left are what you must type into the **robot
configuration on the Driver Station phone** — they have to match exactly, or the
robot won't init.

| Config name | What it is |
|---|---|
| `leftFront`, `leftRear`, `rightFront`, `rightRear` | the 4 mecanum drive motors |
| `rubberBands` | the intake motor |
| `pinpoint` | goBILDA Pinpoint — tracks robot position on the field |
| `Webcam 1` | the camera used to find balls |

No servos yet. Source of truth: [RobotHardwareNames.java](../v1/hardware/RobotHardwareNames.java).

---

## 5. Follow one button press all the way through

This is the best way to learn the codebase. Driver pushes the left stick forward
in [TeleOpMode.java](../v1/opmodes/TeleOpMode.java):

1. **Read it.** `gamepad1.left_stick_y` → a number from -1 to +1.
   (Heads up: the SDK reports **negative** when you push *up*. Everyone trips on this once.)
2. **Clean it up.** `applyTeleOpDrive(...)` runs the stick through four filters:
   - **Deadzone** — sticks don't rest perfectly at 0, so ignore anything under 0.1.
     We use a *circular* deadzone (`Math.hypot(x, y)`) so diagonals feel the same as straight pushes.
   - **Exponential shaping** — `power³`. Small stick movements become *much* smaller
     output, so the driver gets fine control near center but still reaches 100% at full push.
   - **Precision mode** — holding `right_trigger` blends speed down to 30%.
   - **Field-centric** (toggle with `back`) — rotates your stick direction by the robot's
     heading, so "stick up" always means "away from the drivers" even if the robot is
     spun sideways. This is the `cos`/`sin` block.
3. **Send it.** `robot.drive.setTeleOpDrive(y, x, turn)`.
4. **Subsystem acts.** [DriveSubsystem](../v1/subsystems/DriveSubsystem.java) hands those
   three numbers to **Pedro Pathing**, a library that does the mecanum math and sets the
   four motor powers.

Notice what the OpMode never did: it never named a motor.

---

## 6. The ball-seeking code (our newest feature)

[TeleOpMode_ballseek.java](../v1/opmodes/TeleOpMode_ballseek.java) is a normal
TeleOp *plus* a mode where the robot finds a purple ball and drives to it by
itself. Press **A** to toggle seek mode on and off.

### Step 1 — seeing the ball

[VisionService.java](../v1/services/VisionService.java) runs a
`ColorBlobLocatorProcessor` on the webcam. Every frame it:

1. Finds all purple regions in the image (`ColorRange.ARTIFACT_PURPLE`).
2. Throws out ones that are too small, too big, or not round enough
   (thresholds in [VisionConfig.java](../v1/config/VisionConfig.java)).
3. Reports what's left as a `BallTarget` with these fields:

| Field | Meaning |
|---|---|
| `isVisible` | did we find a ball at all |
| `normalizedXError` | **−1 = far left of frame, 0 = dead center, +1 = far right** |
| `radiusPx` | how big it looks → a stand-in for how close it is |
| `circularity` | how ball-shaped it is (1.0 = perfect circle) |

The camera runs at only **320×240**. That's deliberate — it turned out to work
better in dim gym lighting than a higher resolution.

`normalizedXError` is the clever bit. Instead of "the ball is at pixel 214," we
report "the ball is 34% of the way right of center." That number means the same
thing no matter what resolution the camera runs at.

### Step 2 — driving to it (proportional control)

This is the core idea, and it's two lines of real math:

```java
turn    = normalizedXError * TURN_KP;                 // ball is right → turn right
forward = (TARGET_RADIUS - radiusPx) * FORWARD_KP;    // ball looks small → drive forward
```

**Error × gain = response.** The further off you are, the harder you correct;
as you get closer, you automatically ease off. That's a *proportional
controller* (the P in PID), and it's most of what robot control is.

`TURN_KP` and `FORWARD_KP` are the "how aggressive" knobs, in
[TeleOpMode_ballseekConfig.java](../v1/config/TeleOpMode_ballseekConfig.java).
Too low and the robot creeps toward the ball forever; too high and it overshoots
and wobbles back and forth. Tuning them is a great first job.

Both outputs get `Range.clip(...)`'d to a max so the robot never lunges.

### Step 3 — the four behaviors

The robot is always in one of these. Read them in this order and the file makes sense:

| Behavior | When | What it does |
|---|---|---|
| **TRACK** | ball visible | turn toward it + drive at it (the math above) |
| **HOLD** | lost it < 0.5 s ago | *keep doing whatever it was doing* |
| **SEARCH** | lost it > 0.5 s ago | stop, spin in place looking for one |
| **AUTO&nbsp;FORWARD** | ball was centered AND close | ignore the camera, drive straight 2 s to scoop it |

Two of these exist because of problems we hit on the field:

- **HOLD** fixes *jitter*. Blob detection drops out for a single frame all the
  time. Without HOLD, the robot would stutter between "chase" and "search"
  several times a second. So when the ball vanishes we just don't change
  anything for half a second and wait for it to come back.
- **AUTO FORWARD** fixes the *last few inches*. When the robot gets close, the
  ball slides out of the camera's view below the lens — the robot goes blind
  exactly when it's about to succeed. So once we're centered and close, we stop
  trusting the camera and commit to driving straight for a fixed time.

And one small trick worth knowing — **direction memory**:

```java
turn = Math.signum(turn) * BALL_SEARCH_TURN;
```

`signum` returns just the sign (+1 or −1). `turn` is a **field**, not a local
variable, so it still holds the last steering command from when we could see the
ball. If the ball drifted off the right edge, we were turning right, so we keep
searching rightward. Before this, the robot would spin the wrong way and take
a full rotation to find a ball that was six inches off-screen.

---

## 7. `config/` and live tuning

Every file in [`v1/config/`](../v1/config/) is nothing but named numbers:

```java
@Config
public class DriveConfig {
    public static double TELEOP_SPEED_SCALE = 1.0;
    public static double TELEOP_PRECISION_SCALE = 0.3;
}
```

The `@Config` annotation is **FtcDashboard**. Connect a laptop to the robot's
WiFi, open `192.168.43.1:8080/dash`, and you can change these numbers **while
the robot is running** — no rebuild, no re-download. It also streams the camera
feed, which is how you'd check whether the vision filter is actually finding the
ball. This turns a 3-minute tuning cycle into a 3-second one, so use it.

(Note: `TeleOpMode_ballseekConfig` is currently missing its `@Config` annotation,
so it *doesn't* show up in the dashboard yet. Adding it would be a nice tiny PR.)

---

## 8. Two things that will confuse you

**Bulk caching.** The first line of every loop is:

```java
robot.hardware.clearBulkCache();
```

Reading a motor encoder takes ~2 ms over the wire. Read six sensors
individually and you've burned 12 ms of a 20 ms loop. Bulk caching grabs
*everything* in one transaction and serves the rest from memory. `clearBulkCache()`
says "that snapshot is stale, take a fresh one." Forget it and your sensors
silently return the same values forever. It must be **first**, every loop.

**Nothing crashes on missing hardware.** If the webcam is unplugged,
`VisionService` catches it, sets an error message, and the OpMode still runs —
just without vision. Same for the drive. This is on purpose: a robot that runs
in degraded mode can still finish a match. A robot that throws an exception at
init is a forfeit. When you add a subsystem, follow the pattern — null-check and
degrade, don't crash.

---

## 9. Your first change

1. **Get it building.** Open the repo in Android Studio, plug in a phone or Control
   Hub, hit Run. Confirm "V1 TeleOp" shows up in the TeleOp list on the Driver Station.
2. **Change a number.** Set `TELEOP_PRECISION_SCALE` to `0.15` in
   [DriveConfig.java](../v1/config/DriveConfig.java). Drive with the trigger held.
   Feel the difference. You just changed robot behavior.
3. **Add telemetry.** Put a `telemetry.addData("My Value", something);` in a loop
   and watch it on the Driver Station. This is how you debug — there's no
   breakpoint debugger while the robot is driving.
4. **Tune the ball seek.** Take `BALL_SEEK_TURN_KP` up and down and watch what
   overshoot actually looks like.
5. **Then write something new.** Ask a lead for a subsystem to own.

Rules for your first PR:
- Numbers go in `config/`, never inline in an OpMode.
- Hardware names go in `RobotHardwareNames`, never as a string literal in a subsystem.
- Branch off `main`, open a PR, don't push to `main`.

---

## 10. Glossary

| Term | Meaning |
|---|---|
| **OpMode** | a program you run on the robot; TeleOp or Autonomous |
| **Driver Station** | the phone/tablet the drivers hold; picks and runs OpModes |
| **Control Hub** | the computer on the robot that actually runs your code |
| **hardwareMap** | the SDK's lookup table from config names to real devices |
| **Telemetry** | text printed to the Driver Station screen — your `println` |
| **Mecanum** | wheels with angled rollers; lets the robot strafe sideways |
| **Odometry** | tracking position by measuring wheel/pod rotation (our `pinpoint`) |
| **Pose** | robot position + heading: (x, y, angle) |
| **Pedro Pathing** | the library that drives the robot along paths |
| **FtcDashboard** | laptop web UI for live tuning + camera view |
| **P controller** | `output = error × gain` — correct proportionally to how wrong you are |
| **Subsystem** | one class owning one mechanism |
| **Edge detection** | reacting to a button *becoming* pressed, not *being* pressed |

---

## 11. Where to go next

- [ROBOT_ARCHITECTURE_GUIDE.md](ROBOT_ARCHITECTURE_GUIDE.md) — the full rules, FSM
  patterns, code templates, and the "what NOT to do" list.
- [TeleOpMode.java](../v1/opmodes/TeleOpMode.java) — simplest complete OpMode.
- [TeleOpMode_ballseek.java](../v1/opmodes/TeleOpMode_ballseek.java) — vision + autonomous behavior.
- [DriveForwardOneAuto.java](../v1/opmodes/auto/DriveForwardOneAuto.java) — simplest autonomous.
- Official FTC docs: <https://ftc-docs.firstinspires.org>
