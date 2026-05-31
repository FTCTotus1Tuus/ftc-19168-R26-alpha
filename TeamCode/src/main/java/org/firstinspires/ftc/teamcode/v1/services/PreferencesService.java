package org.firstinspires.ftc.teamcode.v1.services;

import android.content.Context;
import android.content.SharedPreferences;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * PreferencesService — persistence boundary for handoff data between OpModes.
 *
 * Planned responsibilities:
 * - store autonomous outputs (e.g., alliance/pose)
 * - restore values in TeleOp initialization
 * - keep storage backend isolated from subsystems and OpModes
 */
public class PreferencesService {

    public static final String ALLIANCE_UNKNOWN = "unknown";
    public static final String ALLIANCE_RED = "red";
    public static final String ALLIANCE_BLUE = "blue";

    private static final String PREFS_NAME = "ftc19168_v1_preferences";
    private static final String KEY_HAS_POSE = "handoff_has_pose";
    private static final String KEY_POSE_X = "handoff_pose_x";
    private static final String KEY_POSE_Y = "handoff_pose_y";
    private static final String KEY_POSE_HEADING = "handoff_pose_heading";
    private static final String KEY_ALLIANCE = "handoff_alliance";

    private SharedPreferences sharedPreferences;

    /** Initializes preference storage if needed. */
    public void initialize() {
        if (sharedPreferences != null) {
            return;
        }

        Context context = AppUtil.getDefContext();
        sharedPreferences = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);

        // TODO: add additional season handoff keys here (e.g., park side, scored game elements).
    }

    /** Flushes/saves preference state if needed. */
    public void stop() {
        // No-op for SharedPreferences.apply(); keep this method as an extension point.
    }

    public void saveAllianceColor(String allianceColor) {
        SharedPreferences prefs = getPrefsOrNull();
        if (prefs == null) {
            return;
        }

        String normalized = normalizeAlliance(allianceColor);
        prefs.edit().putString(KEY_ALLIANCE, normalized).apply();
    }

    public String getAllianceColor() {
        SharedPreferences prefs = getPrefsOrNull();
        if (prefs == null) {
            return ALLIANCE_UNKNOWN;
        }
        return normalizeAlliance(prefs.getString(KEY_ALLIANCE, ALLIANCE_UNKNOWN));
    }

    public void savePose(Pose pose) {
        SharedPreferences prefs = getPrefsOrNull();
        if (prefs == null || pose == null) {
            return;
        }

        prefs.edit()
                .putBoolean(KEY_HAS_POSE, true)
                .putLong(KEY_POSE_X, Double.doubleToRawLongBits(pose.getX()))
                .putLong(KEY_POSE_Y, Double.doubleToRawLongBits(pose.getY()))
                .putLong(KEY_POSE_HEADING, Double.doubleToRawLongBits(pose.getHeading()))
                .apply();
    }

    public Pose getSavedPose() {
        SharedPreferences prefs = getPrefsOrNull();
        if (prefs == null || !prefs.getBoolean(KEY_HAS_POSE, false)) {
            return null;
        }

        double x = Double.longBitsToDouble(prefs.getLong(KEY_POSE_X, Double.doubleToRawLongBits(0.0)));
        double y = Double.longBitsToDouble(prefs.getLong(KEY_POSE_Y, Double.doubleToRawLongBits(0.0)));
        double heading = Double.longBitsToDouble(prefs.getLong(KEY_POSE_HEADING, Double.doubleToRawLongBits(0.0)));
        return new Pose(x, y, heading);
    }

    public void clearSavedPose() {
        SharedPreferences prefs = getPrefsOrNull();
        if (prefs == null) {
            return;
        }

        prefs.edit()
                .remove(KEY_HAS_POSE)
                .remove(KEY_POSE_X)
                .remove(KEY_POSE_Y)
                .remove(KEY_POSE_HEADING)
                .apply();
    }

    private SharedPreferences getPrefsOrNull() {
        return sharedPreferences;
    }

    private static String normalizeAlliance(String allianceColor) {
        if (ALLIANCE_RED.equalsIgnoreCase(allianceColor)) {
            return ALLIANCE_RED;
        }
        if (ALLIANCE_BLUE.equalsIgnoreCase(allianceColor)) {
            return ALLIANCE_BLUE;
        }
        return ALLIANCE_UNKNOWN;
    }
}

