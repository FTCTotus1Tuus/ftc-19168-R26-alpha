# New Member Guide — FTC 19168 Robot Code

**Read [START_HERE.md](START_HERE.md) first.** It's 15 minutes and explains the
big ideas: the loop, the kitchen, pressed-vs-held, and how the ball chase works.

This doc is step two: **the guided tour of the actual code.** Same ideas, but now
with real file names, so that when you open the project, you know what you're
looking at. When you're comfortable here, `ROBOT_ARCHITECTURE_GUIDE.md` has the
full rules.

---

## 1. The robot in one paragraph

Four wheels that can slide sideways, a spinning-rubber-band intake that pulls
balls in, a camera that spots purple balls, and a sensor that tracks where the
robot is on the field. The code is organized so that one class talks to the
hardware, each mechanism gets its own class, and the programs the drivers run
just give orders. Nobody reaches around anybody else.

---

## 2. OpModes — the programs you actually run

An **OpMode** is one runnable robot program. The Driver Station tablet shows a
menu of them; the drivers pick one and run it. Two kinds:

| | Annotation in code | When |
|---|---|---|
| Autonomous | `@Autonomous` | first 30 seconds — robot is on its own |
| TeleOp | `@TeleOp` | rest of the match — humans drive |

Every OpMode has the same life story:

```java
// 1. Everything up here runs when the driver presses INIT — set things up
waitForStart();          // 2. ...then the code stands at this line, waiting
while (opModeIsActive()) {
    // 3. Driver pressed PLAY — this loop now runs ~50 times a second
}
// 4. Match over or STOP pressed — clean up
```

That loop is the cruise-control idea from START_HERE: each pass answers
*"what power should each motor get, right now?"* — and nothing else.

Ours live in [`v1/opmodes/`](../v1/opmodes/). The simplest complete one is
[TeleOpMode.java](../v1/opmodes/TeleOpMode.java) — start there.

---

## 3. The folders, and the one rule

START_HERE gave you the kitchen. Here's the same kitchen with real names:

| Kitchen | Folder | What's inside |
|---|---|---|
| The waiter | `opmodes/` | reads gamepads, gives orders, never cooks |
| The cooks | `subsystems/` | `DriveSubsystem`, `IntakeSubsystem` — one job each |
| The pantry | `hardware/` | `RobotHardware` — the only class allowed to touch motors |
| The recipe card | `config/` | every tunable number, no logic |
| The specialists | `services/` | camera (`VisionService`), position tracking, saved settings |

The rule that makes it work:

> An OpMode never touches a motor. It calls a cook.
> A cook never hunts for hardware. It asks the pantry.
> Nobody memorizes a number. Numbers live on the recipe card.

**Why?** When the intake motor gets rewired to a different port, you change one
word in [RobotHardwareNames.java](../v1/hardware/RobotHardwareNames.java) —
instead of hunting through five files.

**How an OpMode reaches everything:** every OpMode gets one object called
`robot` that holds the whole kitchen. So the code you write looks like:

```java
robot.drive.setTeleOpDrive(forward, strafe, turn);   // tell the drive cook
robot.intake.start();                                 // tell the intake cook
robot.vision.getBallTarget();                         // ask the camera
```

That's honestly most of the API you need for months.

---

## 4. Our hardware — seven things

The names below are typed into the **robot configuration on the Driver Station**
by hand. Code and tablet must match *exactly* — one wrong letter and that device
isn't found. (What happens then is a good story — see section 8.)

| Config name | What it is |
|---|---|
| `leftFront`, `leftRear`, `rightFront`, `rightRear` | the 4 drive motors |
| `rubberBands` | the intake motor |
| `pinpoint` | the position sensor (odometry) |
| `Webcam 1` | the camera |

No servos yet. Source of truth:
[RobotHardwareNames.java](../v1/hardware/RobotHardwareNames.java).

---

## 5. What happens when the driver pushes the stick

Best way to learn the codebase: follow one push of the left stick all the way
through [TeleOpMode.java](../v1/opmodes/TeleOpMode.java). The stick gives us a
number from −1 to +1 — and then it goes through four filters before any motor
sees it. Each filter fixes a real problem:

**Filter 1 — the deadzone.** A gamepad stick never rests at exactly zero; it
wobbles slightly, like a steering wheel with a little play in it. Without this
filter the robot would creep across the floor on its own. So: any input smaller
than 0.1, treat as zero.

**Filter 2 — the gentle-start curve.** We cube the input (`power³`). Sounds odd,
does something lovely: half-stick becomes 0.5³ = 0.13 — much gentler — while
full stick is still 1.0³ = 1.0, full power. It's a gas pedal where the first
inch is soft: fine control for lining up, full speed still there when you floor
it.

**Filter 3 — slow mode.** Hold the right trigger and everything scales down to
30%. It blends — half-pressed trigger is half the slowdown. For precise moves
near the scoring zone.

**Filter 4 — field-centric (the fancy one).** Anyone who's driven an RC car
knows: when the car drives toward you, left and right swap, and everyone
crashes. Same with the robot. Field-centric fixes it — the robot knows which way
it's facing (that's the `pinpoint` sensor), so "stick up" can always mean *away
from the drivers*, no matter which way the robot is turned. The driver toggles
it with the **back** button. There's some rotation math in the code
(`Math.cos`/`Math.sin`) — you don't need to understand it, just what it's for.

After the four filters, the OpMode hands three numbers to
`robot.drive.setTeleOpDrive(...)` — forward, sideways, turn — and the
[DriveSubsystem](../v1/subsystems/DriveSubsystem.java) passes them to a library
called **Pedro Pathing** that does the wheel math.

Notice what never happened: the OpMode never named a motor.

**One gotcha everyone hits once:** the FTC SDK reports the stick as **negative
when you push up**. Yes, really. You'll see minus signs handling it.

---

## 6. The ball chase, one level deeper

START_HERE told the story. Here's where each piece lives.

### The camera's report

[VisionService.java](../v1/services/VisionService.java) looks for purple in
every camera frame, throws out blobs that are too small / too big / not round
enough, and reports the best one as a `BallTarget`:

| Field | Meaning |
|---|---|
| `isVisible` | did we find a ball at all |
| `normalizedXError` | where it is: **−1 far left, 0 dead center, +1 far right** |
| `radiusPx` | how big it looks — big means close |
| `circularity` | how round it is (1.0 = perfect circle) |

`normalizedXError` is the clever bit. Instead of "the ball is at pixel 214," we
say "the ball is 34% of the way to the right edge." A percentage means the same
thing no matter what resolution the camera runs — it's the difference between
"third house on the left" and "house number 214," when the street might get
renumbered.

(The camera runs at only 320×240 on purpose — a teammate discovered low
resolution actually sees the ball *better* in a dim gym.)

### The chase math

Two lines in [TeleOpMode_ballseek.java](../v1/opmodes/TeleOpMode_ballseek.java),
and they're the catching-up-to-a-friend idea from START_HERE:

```java
turn    = normalizedXError * TURN_KP;                 // more off-center → turn harder
forward = (TARGET_RADIUS - radiusPx) * FORWARD_KP;    // looks smaller → drive harder
```

The `KP` numbers are the "how aggressive" knobs, and they live in
[TeleOpMode_ballseekConfig.java](../v1/config/TeleOpMode_ballseekConfig.java).
Nobody calculated them — someone guessed, watched the robot, and adjusted until
it looked right. **That's normal.** Numbers like these can't be computed; they
depend on robot weight, floor grip, and battery. Both results also get capped
(`Range.clip`) so the robot never lunges.

### The four behaviors

Also from START_HERE — TRACK, HOLD, SEARCH, AUTO-FORWARD. The code for all four
is in the same file, lines ~110–170. Worth finding each one; the file makes
sense once you know the four moods exist.

### One trick worth admiring: direction memory

When the ball slides off the *right* edge of the screen, which way should the
robot search? Right, obviously — that's where it went. The code remembers this
with one line:

```java
turn = Math.signum(turn) * BALL_SEARCH_TURN;
```

`Math.signum` keeps only the sign of a number: `+0.24` becomes `+1`, `-0.3`
becomes `-1`. And since `turn` is a whiteboard variable (it survives between
passes), it still holds the *last* steering command from when the ball was
visible. So the robot reads its own last move — "I was turning right" — and
keeps searching that way. Before this line existed, the robot would sometimes
spin the wrong direction and take a full circle to find a ball six inches
off-screen.

---

## 7. Changing numbers without rebuilding

Every file in [`v1/config/`](../v1/config/) is just named numbers. The
`@Config` tag on those classes connects them to **FtcDashboard** — a webpage
served by the robot itself. Laptop on the robot's WiFi, open:

```
192.168.43.1:8080/dash
```

Every `@Config` number appears in a form. Edit one and it changes **inside the
running robot, instantly** — no rebuild, no redeploy. It also shows the live
camera feed with the vision circles drawn on it. This turns a 2-minute tuning
cycle into a 2-second one; it's the most powerful tool you'll use.

**The trap everyone falls into once:** dashboard changes live only in the
robot's memory. Restart the robot and they're gone. So: tune live → write the
good number down → **type it into the config file and commit it.** Forget step
three and tomorrow's robot mysteriously drives like last week's.

---

## 8. Two house rules, and why

**`clearBulkCache()` is the first line of every loop.** Grocery-run rule (see
START_HERE): we fetch all sensor readings in one trip and reuse them for the
rest of the pass. This line says "old trip's groceries are stale — go again."
Forget it, and every sensor quietly reports the same frozen numbers forever.

**Broken hardware must not crash the robot.** Unplug the camera and the robot
still drives — `VisionService` notices, warns, and carries on. A robot missing
one part can still play; a robot that crashes at INIT forfeits. When you add
hardware, copy this pattern: check for null, degrade, don't die. (You'll find
one spot in the code that *doesn't* follow this rule — the intake. Finding and
fixing it is a classic first task. Ask.)

---

## 9. Your first week

1. **Build and deploy.** Open the project in Android Studio, get it onto the
   robot, confirm **"V1 TeleOp"** appears on the Driver Station.
2. **Change one number and feel it.** In
   [DriveConfig.java](../v1/config/DriveConfig.java), change
   `TELEOP_PRECISION_SCALE` from `0.3` to `0.15`. Drive with the trigger held —
   slow mode is now twice as slow. You changed how the robot handles with one
   number. (Put it back after.)
3. **Print something.** Add `telemetry.addData("Hello", 123);` inside a loop and
   see it on the Driver Station. This is your `println` — you can't pause a
   moving robot in a debugger.
4. **Watch the ball chase think.** Run "V1 TeleOp_ballseek," press **A**, and
   move a ball around in front of the camera while watching the numbers change
   on screen.
5. **Ask for a real task.** There's a list of small, real fixes waiting — sized
   exactly for a first pull request.

**Rules for that first pull request:**
- Numbers go in `config/`, never typed into the middle of an OpMode.
- Hardware names go in `RobotHardwareNames`, never as quoted strings elsewhere.
- Work on a branch, open a PR. Never push straight to `main`.

---

## 10. Words people will say at you

Beyond the ones in START_HERE:

| Term | Meaning |
|---|---|
| **hardwareMap** | the SDK's lookup table: config name → real device |
| **Pedro Pathing** | the library that does our wheel math and path driving |
| **FtcDashboard** | the live-tuning webpage from section 7 |
| **P controller** | the "correct in proportion to how wrong you are" idea |
| **Edge detection** | acting when a button *becomes* pressed, not while it *is* — the raised-hand game |
| **Blob** | what the camera calls a connected patch of one color |
| **PathChain** | a pre-planned route for autonomous, built for Pedro to follow |
| **Deadzone** | the "ignore tiny stick wobble" filter |

---

## 11. Where to go next

- [ROBOT_ARCHITECTURE_GUIDE.md](ROBOT_ARCHITECTURE_GUIDE.md) — the full rulebook: templates, state machines, what NOT to do.
- [TeleOpMode.java](../v1/opmodes/TeleOpMode.java) — simplest complete OpMode. Read top to bottom once.
- [TeleOpMode_ballseek.java](../v1/opmodes/TeleOpMode_ballseek.java) — the ball chase, now that you know its four moods.
- [DriveForwardOneAuto.java](../v1/opmodes/auto/DriveForwardOneAuto.java) — our (so far only) autonomous.
- Official FTC docs: <https://ftc-docs.firstinspires.org>
