package org.firstinspires.ftc.teamcode.v1.services;

/**
 * PreferencesService — persistence boundary for handoff data between OpModes.
 *
 * Planned responsibilities:
 * - store autonomous outputs (e.g., alliance/pose)
 * - restore values in TeleOp initialization
 * - keep storage backend isolated from subsystems and OpModes
 */
public class PreferencesService {

    /** Initializes preference storage if needed. */
    public void initialize() {
        // TODO: wire FTC blackboard / SharedPreferences persistence.
    }

    /** Flushes/saves preference state if needed. */
    public void stop() {
        // TODO: commit pending writes if future implementation requires it.
    }
}

