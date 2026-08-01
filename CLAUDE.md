# CLAUDE.md — FTC Team 19168 (DECODE 2025-26 season)

FTC robot code. FTC SDK 11.1, Pedro Pathing 2.1.2, FtcDashboard 0.6.0. Runs on a REV
Control Hub (Android); deployed from Android Studio over USB. All team code lives in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/v1/` — the versioned package is
deliberate; don't create code outside `v1/` without discussion.

## Build & deploy

- Android Studio project; the team module is `TeamCode`.
- Build variant: use **`standardDebug`** (the `panelsTuning` flavor swaps FtcDashboard
  for the Panels tuning UI — don't switch variants casually).
- Code cannot run on a laptop. The only laptop-verifiable code is pure-Java logic with
  unit tests. Everything else is verified on the robot — treat "compiles" as the start,
  not the end.

## Package layout

| Package | Contents | Rule |
|---|---|---|
| `v1/opmodes/` | TeleOp + auto OpModes | Read gamepads, call subsystems. Never touch motors. |
| `v1/core/` | `RobotContainer` | Composition root; OpModes access everything as `robot.*` |
| `v1/subsystems/` | `DriveSubsystem`, `IntakeSubsystem` | One mechanism each; get devices from `RobotHardware` |
| `v1/hardware/` | `RobotHardware`, `RobotHardwareNames` | **The only place `hardwareMap.get()` is allowed** |
| `v1/services/` | Vision, localization, preferences, Pedro factory | Non-mechanism capabilities |
| `v1/config/` | `@Config` constant classes | All tunable numbers. No logic. |
| `v1/testing/` | Pedro tuning suite, diagnostics | Not match code |

## Hard rules

1. **Hardware access:** only `RobotHardware` calls `hardwareMap.get()`. Device names are
   constants in `RobotHardwareNames` — never string literals elsewhere. Names must match
   the Driver Station robot configuration exactly.
2. **No magic numbers in logic.** Every tunable lives in a `v1/config/` class as a
   `public static` (non-final) field, annotated `@Config` so it's live-tunable from
   FtcDashboard (`192.168.43.1:8080/dash`). Dashboard edits don't persist — tuned values
   must be typed back into the config file and committed.
3. **Fail-safe init.** Missing hardware must never crash an OpMode. Wrap lookups
   (see `RobotHardware.tryGetMotor`), null-guard usage, degrade gracefully. A robot
   missing a part can still play a match; a robot that throws at INIT forfeits.
4. **Loop discipline.** `robot.hardware.clearBulkCache()` is the first call in every
   loop iteration (hubs are in MANUAL bulk-cache mode — forgetting this freezes all
   sensor reads). The Pedro follower's `update()` must run exactly once per loop.
5. **Buttons:** anything that should fire once per press uses the rising-edge pattern
   (`pressed && !prevPressed`, with `prev` updated after — see the `A`/`back` handling
   in the TeleOps). Never act on button level for toggles.
6. **Timing:** `ElapsedTime` checked per-loop plus a boolean flag. Never `Thread.sleep`
   or blocking waits inside an OpMode loop.
7. **State machines** for multi-step behavior — see the FSM template in
   `docs/ROBOT_ARCHITECTURE_GUIDE.md`.

## Sharp edges (true today — verify before relying on)

- `applyTeleOpDrive(...)` is **duplicated verbatim** in `TeleOpMode` and
  `TeleOpMode_ballseek`. Any change to one must be made in both until it's unified.
- Turn is **double-scaled**: OpModes multiply by `TELEOP_ROTATION_SCALE` and
  `DriveSubsystem.setTeleOpDrive` multiplies by it again. All current tuning (driver
  feel + ball-seek gains) was done on top of this. Do NOT remove one multiply as a
  drive-by cleanup — it changes robot behavior ~43% and requires a field retune
  (ball-seek turn values × 0.7 to preserve behavior).
- `TeleOpMode_ballseekConfig` is missing `@Config` (only config class without it).
- Camera resolution exists in two places: hardcoded `Size(320, 240)` in
  `VisionService` and `VisionConfig.BALL_CAMERA_WIDTH_PX`. They must agree or
  `normalizedXError` silently skews. 320×240 is deliberate (better low-light blob
  detection) — don't raise it without testing.
- Drive motor directions are set in `PedroPathingConstants` (Pedro), not
  `RobotHardware`. Those two files must stay consistent.
- `AGENTS.md` predates the vision/intake/ball-seek work; this file supersedes it where
  they disagree.

## Verification reality

- Telemetry is the debugger — there are no breakpoints on a moving robot.
- FtcDashboard streams the camera with vision overlays; use it to verify blob detection.
- Any change to drive feel, gains, or vision thresholds needs a robot test before it
  counts as done. Say so explicitly when delivering such changes untested.

## Git

- Branch per change; PR to `FTCTotus1Tuus/ftc-19168-R26-alpha` `main`; never push
  `main` directly. PRs get human review before merge.

## Onboarding docs

`teamcode/docs/START_HERE.md` (concepts, analogies) →
`teamcode/docs/NEW_MEMBER_GUIDE.md` (code tour) →
`teamcode/docs/ROBOT_ARCHITECTURE_GUIDE.md` (full rulebook).
