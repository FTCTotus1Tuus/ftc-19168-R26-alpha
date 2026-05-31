# AGENTS.md – AI Coding Agent Guide
*Last updated: May 29, 2026 — revisit when new subsystems or libraries are added.*

## Project Overview
FTC Team 19168 robot controller app for the DECODE (2025-2026) season. Runs on Android (minSdk 24, compileSdk 35, Java 1.8 source/target). Built with **Gradle** in Android Studio Ladybug (2024.2+).

Two modules:
- `FtcRobotController/` – upstream FTC SDK (**do not edit**)
- `TeamCode/` – all team code under `org.firstinspires.ftc.teamcode`

## Current State
Skeleton robot with full architecture in place — these files exist today:

**Root package** (`org.firstinspires.ftc.teamcode`):
- `docs/ROBOT_ARCHITECTURE_GUIDE.md` – team architecture, patterns, and conventions

**`v1/` package** (active architecture):
- `v1/hardware/RobotHardwareNames.java` – canonical hardware config string constants
- `v1/services/PedroPathingConstants.java` – Pedro Pathing follower builder + runtime motor-name resolution
- `v1/testing/PedroTeleOp.java` – original flat TeleOp, kept as diagnostic reference (`@Disabled`)
- `v1/hardware/RobotHardware.java` – bulk caching setup; maps all non-Pedro devices
- `v1/subsystems/DriveSubsystem.java` – wraps Pedro Pathing `Follower`; exposes `setTeleOpDrive()`, `getPose()`
- `v1/core/RobotContainer.java` – composition root; owns all subsystems
- `v1/opmodes/RobotOpMode.java` – thin `LinearOpMode` base class
- `v1/opmodes/TeleOpMode.java` – active driver-control OpMode

## Architecture
See the full guide for class hierarchy, package layout, code templates, startup flow, and how to add subsystems:

> `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/ROBOT_ARCHITECTURE_GUIDE.md`

## Hardware Device Names
Names must exactly match the robot config file. Canonical names live in `v1/hardware/RobotHardwareNames.java`:
```java
"Right-front", "Right-rear", "Left-rear", "Left-front"  // drive motors — kebab-case, capital first word
"pinpoint"                                               // GoBilda Pinpoint I2C device
```

## OpMode Registration
Remove `@Disabled` to make an OpMode appear on the Driver Station:
```java
@TeleOp(name = "TeleOp", group = "DriverControl")
// @Disabled  ← delete this line to activate
```

## Key Dependencies (`build.dependencies.gradle`)
- `com.pedropathing:ftc:2.1.2` — path following + TeleOp drive (`maven { url = 'https://maven.brott.dev/' }`)
- `GoBildaPinpointDriver` — bundled in FTC SDK 10.3+; no extra Gradle dependency needed
- FTC SDK 11.1.0 (`org.firstinspires.ftc:*`)
- `com.acmerobotics.dashboard:dashboard:0.6.0` — FtcDashboard live tuning UI (`maven { url = "https://maven.brott.dev/" }`)

**Note:** This team uses only native FTC SDK patterns for performance (bulk caching via `LynxModule.BulkCachingMode.MANUAL`) and control (custom PIDF calculator + @Config). No third-party control or hardware-optimization libraries — see `ROBOT_ARCHITECTURE_GUIDE.md` §14.

## Agent Behavior Rules
- **Ask clarifying questions before making big assumptions or going down rabbit holes.** If a request is ambiguous or could be interpreted multiple ways, ask first.

## Commit Message Output Convention
- When the user asks for a commit message, return the commit title and body in **one single copy/paste-ready text block**.
- Do not split title/body across multiple disconnected blocks unless the user explicitly asks for alternatives.

## Versioning Rules
- Keep each physical robot version in its own top-level package: `v1/`, `v2/`, etc.
- Keep hardware-bound code version-specific (`RobotHardwareNames`, `RobotHardware`, robot-specific subsystem implementations, robot-specific constants).
- Only move code to a shared `common/` package when it is proven robot-agnostic and used by both versions.
- Do not split into separate repositories for the same season unless versions are operationally independent and share little or no code.
- For Driver Station clarity, rely on OpMode annotations (not Java package names):
  - use explicit names like `V1 TeleOp`, `V2 TeleOp`, `V1 Left Auto`
  - use explicit groups like `v1`, `v2`, `Auto-v1`, `Auto-v2`
- Keep unfinished or risky version branches disabled with `@Disabled` until they are field-tested.

## Build & Deploy
```bash
./gradlew :TeamCode:assembleDebug   # build debug APK from project root
# Deploy via Android Studio Run button (ADB to Control Hub or phone)
# No unit tests — all validation is done on physical hardware
```
