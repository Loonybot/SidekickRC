package com.loonybot.sidekick;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Hooks for Sidekick testing.
 */
public class SkTest {
    public static void start(OpMode opMode, Context appContext, boolean isSimulating) {
//        Globals.sidekick.start(appContext, isSimulating);
//        Globals.sidekick.onOpModePreInit(opMode);
    }
    public static void end(OpMode opMode) {
//        Globals.sidekick.onOpModePostStop(opMode);
    }
    public static <T> T objectWrap(T original, String deviceName) {
        // Disable the cache for perf testing:
        return null; // @@@ Globals.capture.wrapHardwareDevice(original, deviceName, null);
    }
}
