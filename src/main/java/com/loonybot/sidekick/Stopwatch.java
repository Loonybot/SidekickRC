/// Sidekick object for timing stuff.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

/**
 * Sidekick object to measure time intervals.
 */
@SuppressWarnings({"unused"})
public class Stopwatch {
    Capture capture; // If non-null, this is the associated capture object
    int identifier; // Internal identifier for this stopwatch instance

    /// This is a package-private constructor. Call [Sk#createStopwatch] instead to create your
    /// stopwatch.
    Stopwatch(Capture capture, String name) {
        this.capture = capture;
        if (capture != null) {
            identifier = capture.createStopwatch(name);
        }
    }

    /**
     * Start the stopwatch. If the stopwatch was already started, it will be automatically ended and
     * started again. E.g., calling start(), start(), stop() will record two intervals.
     */
    public void start() {
        if (capture != null) {
            capture.startStopwatch(identifier);
        }
    }

    /**
     * Stop the stopwatch and record the resulting duration.
     */
    public void stop() {
        if (capture != null) {
            capture.stopStopwatch(identifier);
        }
    }
}