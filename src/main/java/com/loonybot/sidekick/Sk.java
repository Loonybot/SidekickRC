/// Sidekick public APIs.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Use this {@link Sk} class to configure Sidekick and add capture data.
 */
@SuppressWarnings({"unused"})
public class Sk {
    // Default value for Sk.beginNoMovement():
    public static final double DEFAULT_ROTATION_RATE_TOLERANCE = 1.0; // Degrees/s

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// Sidekick data structures.
    /**
     * Interface for providing a serializer to Sidekick via
     * {@link Sk#registerSerializer(Object, String, Serializer) Sk.config.registerSerializer}
     * for class types it doesn't natively support.
     */
    public interface Serializer<T> {
        /**
         * Sidekick will call this method while the opMode runs to convert objects of the
         * specified type to an array of objects that Sidekick can recognize and serialize.
         *
         * @param value The object instance to serialize; guaranteed non-null.
         * @return An array of objects that represent the values of the instance being serialized.
         * The returned objects can have any types, but the count of objects and their types
         * must never vary, and can never be null, otherwise the opMode will fail. It's fine for
         * an object to be an array that can have varied lengths, however. As an example, a Pose2D
         * might return {@code new Double[] {x, y, heading}}.
         */
        Object[] serialize(T value);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// To invoke the following Sidekick configuration APIs before the first opMode begins, call
    /// them from an @OnCreate method in a custom class, like this:
    ///        @SuppressWarnings("unused")
    ///        class ConfigureSidekick {
    ///            @OnCreate
    ///            public static void onCreate(Context context) {
    ///                Sk.enable(false); // Temporarily disable Sidekick
    ///            }
    ///        }
    /// You only need one copy of this code anywhere in your project. It will affect all of your
    /// opModes. Always BE VERY CAREFUL when changing code in any onCreate() method because, if
    /// it crashes, your robot will continually reboot and you'll have to use the REV Hardware
    /// Client to reflash the system.

    /**
     * Enable or disable Sidekick. This must be called from an @OnCreate method. A side effect
     * is that this removes the enable/disable opMode that Sidekick adds.
     */
    public static void enable(boolean enable) {
        ApiRequestedState state = enable ? ApiRequestedState.ENABLED : ApiRequestedState.DISABLED;
        if (!Sidekick.setApiRequestedState(state)) {
            throw new IllegalStateException("Sk.enable() can ony be called from an @OnCreate method.");
        }
    }

    /**
     * Sidekick automatically captures data every time an opMode is run. This method allows
     * you to specify how how long to keep that data before it's automatically deleted.
     * Must be called from an @OnCreate method.
     *
     * @param days Delete Sidekick data from the robot after this many days.
     */
    public static void setRetentionDays(int days) {
        if (Sidekick.isEnabled) {
            if (!Sidekick.instance.setRetentionDays(days)) {
                throw new IllegalStateException("Sk.setRetentionDays() can ony be called from an @OnCreate method.");
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// The following can be called from @OnCreate or an opMode.

    /**
     * Suppress specific Sidekick warnings, errors and suggestions.
     *
     * @param issueCodes A list of issue codes to disable. For example, to disable issues
     *                   with the codes 101 and 107, specify
     *                   {@code Sk.suppressIssues(101, 107)}.
     */
    public static void suppressIssues(int... issueCodes) {
        if (Sidekick.isEnabled) {
            Sidekick.instance.suppressIssues(issueCodes);
        }
    }

    /**
     * Register a serializer to enable Sidekick to save complex objects in their component
     * forms for later analysis in the Sidekick app. Think of a serializer as an alternative
     * to a class's toString() method for purposes of Sidekick captures, as it will show all
     * the object's parameters rather than an obtuse string like
     * "org.firstinspires.ftc.robotcore.external.navigation.Pose2D@1b2543ea". It is also faster
     * than any toString() method, consumes less space in the archive file, enables graphing
     * of numeric data, and supports 3rd party objects when you can't modify the code to add
     * a toString() method.
     *
     * @param sample  A sample instance of the type of object to serialize. Sidekick will
     *                call the serializer with this sample to determine the types of the
     *                serialized parameters and pre-compile the serialization process.
     * @param format A string that represents the name and unit of every parameter
     *                   of the serialized result. For example, for a Pose2D it might be
     *                   "x=in, y=in, heading=rad". It's fine not to specify units - for
     *                   example, a Point might be "x=, y=".
     * @param serializer A serializer that implements the {@link Serializer} interface.
     */
    public static <T> void registerSerializer(@NonNull T sample, @NonNull String format, @NonNull Serializer<T> serializer) {
        if (Sidekick.isEnabled) {
            Sidekick.instance.registerSerializer(sample, format, serializer);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// Anything in this section is for testing purposes only.

    /**
     * Don't call this, as it's private for testing purposes. It allows the capture file
     * name to be specified, but it doesn't support companion files like video captures, so
     * you'd lose data.
     *
     * @param fileName The name to be given to the completed capture file.
     */
    public static void _setPrivateCaptureName(String fileName) {
        if (Sidekick.isEnabled) {
            Sidekick.instance.setCaptureName(fileName);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// APIs to be called from an opMode.

    /**
     * Get the original object from a wrapped Sidekick object. Sidekick wraps every object returned
     * from {@link com.qualcomm.robotcore.hardware.HardwareMap#get(String) HardwareMap.get(String)}
     * and the like with a <i>proxy</i> object that has extra code to record the call and its
     * parameters. Unwrapping is useful when you want to use an object without having to record
     * the operations into the archive, or if there is a compatibility problem with Sidekick's
     * wrapping.
     *
     * @param object The object to unwrap; typically a HardwareDevice object.
     * @return The same object, but without Sidekick's wrapping.
     */
    @NonNull public static <T> T unwrap(@NonNull T object) {
        return Capture.unwrap(object);
    }

    /**
     * Record a note into the archive.
     *
     * @param note A note string. It optionally supports {@link String#format(String, Object...)}
     *             formatting for the specified values.
     * @param values Any values to add to the note. It's fine if the note string doesn't have
     *              formatting; the values will be shown at the end of the note.
     */
    public static void note(@NonNull String note, Object... values) {
        if (Capture.instance != null) {
            Capture.instance.note(note, values);
        }
    }

    /**
     * Record an alert.
     *
     * @param alert An identifying alert message. It optionally supports
     *              {@link String#format(String, Object...)} formatting for any supplied values.
     *              It's fine if the string doesn't have formatting; in that case, the values will
     *              be shown at the end.
     * @param values Any values to supplement the alert string.
     */
    public static void alert(@NonNull String alert, Object... values) {
        if (Capture.instance != null) {
            Capture.instance.recordVarargs(VarargsType.ALERT, alert, values);
        }
    }

    /**
     * Evaluate a requirement condition and if it fails, record an error message along with a stack
     * trace.
     *
     * @param condition The condition to check; if it's false, the failure will be recorded.
     * @param requirement An identifying message to be recorded when the requirement fails. It
     *                    optionally supports {@link String#format(String, Object...)} formatting
     *                    for any supplied values. It's fine if the requirement string doesn't
     *                    have formatting; in that case, the values will be shown at the end.
     * @param values Any values to supplement the requirement string.
     */
    public static void require(boolean condition, @NonNull String requirement, Object... values) {
        if (!condition) {
            if (Capture.instance != null) {
                Capture.instance.recordVarargs(VarargsType.REQUIRE, requirement, values);
            }
        }
    }

    /**
     * Record an FTC Dashboard Canvas object in the archive.
     *
     * @param canvas The Dashboard Canvas object with drawing commands to record; must be of type
     *               com.acmerobotics.dashboard.canvas.
     * @throws IllegalArgumentException if the object isn't the right type.
     */
    public static void captureCanvas(@NonNull Object canvas) {
        if (Capture.instance != null) {
            Capture.instance.captureCanvas(canvas);
        }
    }

    /**
     * Mark the beginning/end of a control loop.
     */
    public static void loop() {
        if (Capture.instance != null) {
            Capture.instance.loop();
        }
    }

    /**
     * When you create a new thread, call this so that Sidekick can track its performance.
     *
     * @param threadName The name Sidekick will use to identify the thread.
     */
    public static void registerThreadStart(@NonNull String threadName) {
        if (Sidekick.isEnabled) {
            Sidekick.instance.registerThreadStart(threadName);
        }
    }

    /**
     * Call this if you can before your thread ends so that Sidekick can record how much CPU
     * it consumed.
     */
    public static void registerThreadEnd() {
        if (Sidekick.isEnabled) {
            Sidekick.instance.registerThreadEnd();
        }
    }

    /**
     * Create a stopwatch object for measuring time intervals.
     *
     * @param name Identifier for this stopwatch.
     * @return The resulting stopwatch object.
     */
    public static Stopwatch createStopwatch(@NonNull String name) {
        return new Stopwatch(Capture.instance, name);
    }

    /**
     * Register the robot's current pose so that it can be tracked by Sidekick.
     *
     * @param pose The current pose of the robot.
     */
    public static void pose(@NonNull Pose2D pose) {
        if (Capture.instance != null) {
            Capture.instance.pose(pose);
        }
    }

    /**
     * Call this before performing gyro calibration for devices that requires the robot not to move,
     * such as when calling GoBildaPinpointDriver.recalibrateIMU() or SparkFunOTOS.calibrateImu().
     * In conjunction with Sk.endNoMovement(), this will use the built-in IMU to verify that the
     * robot wasn't moved.
     *
     * @param rotationRateTolerance Maximum rotation rate tolerance, degrees/s.
     * @throws IllegalStateException if no IMU is found.
     */
    public static void beginNoMovement(double rotationRateTolerance) {
        if (Capture.instance != null) {
            if (!Capture.instance.beginNoMovement((float) rotationRateTolerance)) {
                throw new IllegalStateException("An IMU is required to call beginNoMovement(). "
                        + "No IMU was found even though every Control Hub and Expansion Hub has one. "
                        + "Someone deleted the IMU in the Configuration for I2C bus 0.");
            }
        }
    }
    public static void beginNoMovement() {
        beginNoMovement(DEFAULT_ROTATION_RATE_TOLERANCE);
    }

    /**
     * Call this after performing gyro calibration for devices that requires the robot not to move,
     * such as when calling GoBildaPinpointDriver.recalibrateIMU() or SparkFunOTOS.calibrateImu().
     * In conjunction with Sk.beginNoMovement(), this will use the built-in IMU to verify that the
     * robot wasn't moved.
     *
     * @return True the robot was not moved (success), false if it was moved (failure).
     */
    public static boolean endNoMovement() {
        boolean result = true; // Default to success
        if (Capture.instance != null) {
            result = Capture.instance.endNoMovement();
        }
        return result;
    }

}