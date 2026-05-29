# AGENTS.md – AI Coding Agent Guide
*Last updated: May 29, 2026 — revisit when new subsystems or libraries are added.*

## Project Overview
FTC Team 19168 robot controller app for the DECODE (2025-2026) season. Runs on Android (minSdk 24, compileSdk 35, Java 1.8 source/target). Built with **Gradle** in Android Studio Ladybug (2024.2+).

Two modules:
- `FtcRobotController/` – upstream FTC SDK (**do not edit**)
- `TeamCode/` – all team code under `org.firstinspires.ftc.teamcode`

## Current State
Early skeleton robot — these files exist today:
- `RobotHardwareNames.java` – canonical hardware config string constants
- `PedroPathingConstants.java` – Pedro Pathing follower builder + runtime motor-name resolution
- `PedroTeleOp.java` – working TeleOp that drives with the Pedro Pathing follower

## Architecture
See the full guide for class hierarchy, package layout, code templates, startup flow, and how to add subsystems:

> `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/ROBOT_ARCHITECTURE_GUIDE.md`

## Hardware Device Names
Names must exactly match the robot config file. Canonical names live in `RobotHardwareNames.java`:
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

## Build & Deploy
```bash
./gradlew :TeamCode:assembleDebug   # build debug APK from project root
# Deploy via Android Studio Run button (ADB to Control Hub or phone)
# No unit tests — all validation is done on physical hardware
```
