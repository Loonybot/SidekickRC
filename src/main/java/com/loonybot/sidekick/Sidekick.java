/// Sidekick state maintained across opModes and captures.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import androidx.annotation.NonNull;

import com.google.gson.Gson;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.RobocolDatagram;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.network.CallbackResult;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.network.RecvLoopRunnable;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.objenesis.Objenesis;
import org.objenesis.ObjenesisStd;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IllegalFormatException;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.function.BiFunction;

import fi.iki.elonen.NanoWSD;

/// OpMode notification types.
enum OpModeNotification { PRE_INIT, PRE_START, POST_STOP }
enum ApiRequestedState { NOT_CALLED, ENABLED, DISABLED }

/// Class for hooking gamepad callbacks.
class ReceiveLoopCallbacks implements RecvLoopRunnable.RecvLoopCallback {
    /// Safely return the current capture object, if any.
    Capture getCapture() {
        synchronized(Sidekick.sidekickLock) {
            return Capture.instance;
        }
    }
    @Override public CallbackResult gamepadEvent(RobocolDatagram packet) {
        Capture capture = getCapture();
        if (capture != null) {
            capture.gamepadData(packet.getData());
        }
        return CallbackResult.NOT_HANDLED;
    }
    @Override public CallbackResult reportGlobalError(String error, boolean recoverable) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult packetReceived(RobocolDatagram packet) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult peerDiscoveryEvent(RobocolDatagram packet) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult heartbeatEvent(RobocolDatagram packet) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult commandEvent(Command command) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult telemetryEvent(RobocolDatagram packet) { return CallbackResult.NOT_HANDLED; }
    @Override public CallbackResult emptyEvent(RobocolDatagram packet) { return CallbackResult.NOT_HANDLED; }
}

/// Core Sidekick class whose single instantiation persists for the duration of the robot being
/// powered on. Multiple captures may be instantiated in that time.
public class Sidekick implements OpModeManagerNotifier.Notifications {
    /// Package-private class for storing the serializer and its default object.
    static class SerializerInfo<T> {
        Class<?> klass; // Class type that the serializer applies to
        int identifier; // Identifier of this serializer; increments on every one, starts from zero
        Sk.Serializer<T> serializer; // Serializer for this type
        Object[] prototypeOutput; // Serialized sample that gives us the output object types
        String format; // Format describing parameter names and units
        SerializerInfo(Class<?> klass, int identifier, Sk.Serializer<T> serializer, Object[] prototypeOutput, String format) {
            this.klass = klass;
            this.identifier = identifier;
            this.serializer = serializer;
            this.prototypeOutput = prototypeOutput;
            this.format = format;
        }
    }

    // Sidekick uses threads with these names:
    final static String SAMPLE_WORKER_THREAD_NAME = "Sidekick Sample Worker";
    final static String FILE_WORKER_THREAD_NAME = "Sidekick File Worker";
    final static String VIDEO_CAPTURE_THREAD_NAME = "Sidekick Video Capture";

    /// Structure to track non-Sidekick threads that the user has registered.
    static class ThreadRegister {
        int tid; // Linux TID
        String name; // Thread name
        ThreadRegister(int tid, String name) {
            this.tid = tid;
            this.name = name;
        }
    }

    // Run status:
    static ApiRequestedState apiRequestedState = ApiRequestedState.NOT_CALLED; // Was Sk.enable()/disable() called?
    static boolean isInitialized = false; // True if Sidekick has been initialized
    static boolean isEnabled = true; // True if Sidekick has been enabled by the user
    static boolean nextBootEnabled = true; // True if Sidekick will be enabled on the next boot
    static volatile int opModeCount; // Count of user-defined opModes initialized (includes default opModes)
    static CountDownLatch primingDoneSignal = new CountDownLatch(1); // Signals when the FileWorker primes are done
    int lastRunNumber; // Number of opModes run with the current build of the app

    // True constants:
    final static int WEB_SOCKET_PORT = 9090; // Port for the Sidekick web listener
    final static String TAG = "Sidekick"; // Identifier for Logcat error logging
    final static long MIN_STORAGE_MBS = 100; // Always keep this many megabytes free in "/sdcard"
    final static String SD_CARD_PATH = Environment.getExternalStorageDirectory().getPath(); // AKA 'sdcard'
    final static String SUBDIRECTORY = SD_CARD_PATH + "/sidekick"; // Subdirectory for all captures
    final static boolean isPC = !"The Android Project".equals(System.getProperty("java.vm.name"))
            && !Objects.requireNonNull(System.getProperty("java.runtime.name")).contains("Android");

    // Effective constants:
    final static Sidekick instance = new Sidekick(); // Internal sidekick object, always non-null after static init
    final static Gson gson = new Gson(); // For creating JSON files
    final static Objenesis objenesis = new ObjenesisStd(); // ByteBuddy requirement
    final static Object sidekickLock = new Object(); // Master thread lock for Sidekick

    // State:
    FtcEventLoop eventLoop; // For creating our own HardwareMap and controlling opModes
    SampleWorker sampleWorker; // Periodic thread for periodic work
    FileWorker fileWorker; // File writing thread
    boolean oneTimeOpModeInitializationDone; // True if we've done one-time opMode initialization
    boolean isGamepadHooked; // True if the gamepad is successfully hooked
    boolean isDefaultOpMode; // True if the default opMode is active
    BiFunction<Integer, Object, Object> emulatorCallback; // Emulator enlightenment callback, if any
    ProxyBuilder proxyBuilder; // Byte Buddy proxy builder
    String appBuildTime = ""; // Build time of the currently running app
    // Map of serializers for custom types:
    @SuppressWarnings({"rawtypes"}) HashMap<Class<?>, SerializerInfo> serializersMap = new HashMap<>();
    LinkedList<SerializerInfo<?>> serializersList = new LinkedList<>(); // List of all serializers
    Set<Integer> suppressedIssues = new HashSet<>(); // Issue codes that the user wants suppressed
    ReceiveLoopCallbacks receiveLoopCallbacks = new ReceiveLoopCallbacks(); // For Gamepad hooks
    OpModeNotification expectedNotification = OpModeNotification.PRE_INIT; // The expected next OpMode notification
    HashMap<Thread, ThreadRegister> threadRegistry = new HashMap<>(); // Registered threads

    final private NanoWSD server = new NanoWSD(WEB_SOCKET_PORT) {
        @Override protected WebSocket openWebSocket(IHTTPSession handshake) {
            return new Socket(handshake);
        }
    };

    /// Output Logcat messages in a consistent way with varargs.
    static String safeFormat(String format, Object... args) {
        try { return String.format(format, args); } catch (IllegalFormatException ignored) { return format; }
    }
    static void logE(String format, Object... args) { // Error
        Log.e(TAG, safeFormat(format, args));
    }
    static void logW(String format, Object... args) { // Warning
        Log.w(TAG, safeFormat(format, args));
    }
    static void logI(String format, Object... args) { // Info
        Log.i(TAG, safeFormat(format, args));
    }

    /// Register an object serializer.
    <T> void registerSerializer(@NonNull T sample, @NonNull String format, @NonNull Sk.Serializer<T> serializer) {
        synchronized(sidekickLock) {
            Class<?> klass = sample.getClass();
            if (!serializersMap.containsKey(klass)) {
                Object[] prototypeOutput = serializer.serialize(sample);
                int identifier = serializersMap.size();
                SerializerInfo<?> serializerInfo
                        = new SerializerInfo<>(klass, identifier, serializer, prototypeOutput, format);
                serializersMap.put(klass, serializerInfo);
                serializersList.add(serializerInfo);
            }
        }
    }

    /// Set the API-invoked enable/disable state.
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    static boolean setApiRequestedState(ApiRequestedState state) {
        synchronized (sidekickLock) {
            // The API can enable/disable Sidekick only if it hasn't initialized yet:
            apiRequestedState = state;
            return !isInitialized;
        }
    }

    /// Record issues that the user would like to suppress.
    void suppressIssues(int... issueCodes) {
        synchronized(sidekickLock) {
            for (int issueCode: issueCodes) {
                suppressedIssues.add(issueCode);
            }
        }
    }

    /// Set the retention period for automatic Sidekick capture data, in days.
    void setRetentionDays(int days) {
        FileWorker.retentionDays = days;
    }

    /// Register a thread with the thread registry. NOTE: This can't take a thread as an
    /// argument because we can only query the Linux TID from the current thread.
    void registerThreadStart(String threadName) {
        synchronized(sidekickLock) {
            Thread thread = Thread.currentThread();
            ThreadRegister register = new ThreadRegister(android.os.Process.myTid(), threadName);
            threadRegistry.put(thread, register);
            if (Capture.instance != null) {
                Capture.instance.registerThreadTransition(thread, register, true);
            }
        }
    }

    /// Unregister a thread from the thread registry.
    void registerThreadEnd() {
        synchronized(sidekickLock) {
            Thread thread = Thread.currentThread();
            ThreadRegister register = threadRegistry.get(thread);
            if (register != null) {
                if (Capture.instance != null) {
                    Capture.instance.registerThreadTransition(thread, register, false);
                }
                threadRegistry.remove(thread);
            }
        }
    }

    /// Update the persisted property state.
    void updateProperties() {
        Properties props = new Properties();
        props.setProperty("nextBootEnabled", String.valueOf(nextBootEnabled));
        props.setProperty("runNumber", String.valueOf(lastRunNumber));
        props.setProperty("lastAppBuildTime", appBuildTime);

        try (FileOutputStream out = new FileOutputStream(SUBDIRECTORY + "/sidekick.properties")) {
            props.store(out, "Sidekick State");
        } catch (IOException ignored) {
            logE("Couldn't save properties");
        }
    }

    /// Initialize Sidekick. This is the first method to be called at startup.
    @OnCreate public static void onCreate(@SuppressWarnings("unused") Context context) {
        logI(">>> onCreate #1");
    }

    /// Register a listener so that we can hook opMode notifications. This is the second method
    /// the system calls, after onCreate.
    @OnCreateEventLoop @SuppressWarnings("unused") public static void onCreateEventLoop(Context context, FtcEventLoop eventLoop) {
        logI(">>> onCreateEventLoop #2");
        synchronized(sidekickLock) {
            // Load all our persisted state from the last boot (if it exists):
            String lastAppBuildTime = "";
            Properties props = new Properties();
            try (FileInputStream in = new FileInputStream(SUBDIRECTORY + "/sidekick.properties")) {
                props.load(in);
                nextBootEnabled = Boolean.parseBoolean(props.getProperty("nextBootEnabled", "true"));
                isEnabled = nextBootEnabled;
                instance.lastRunNumber = Integer.parseInt(props.getProperty("runNumber", "0"));
                lastAppBuildTime = props.getProperty("lastAppBuildTime", "");

                logI("nextBootEnabled=%b", nextBootEnabled);
                logI("runNumber=%d", instance.lastRunNumber);
                logI("lastAppBuildTime=%s", lastAppBuildTime);
            } catch (IOException ignored) {}

            // If Sk.enable()/disable() has been called, that overrides our saved enable setting:
            if (apiRequestedState != ApiRequestedState.NOT_CALLED) {
                isEnabled = (apiRequestedState == ApiRequestedState.ENABLED);
                // Save the API preference for the next boot, too:
                if (isEnabled != nextBootEnabled) {
                    nextBootEnabled = isEnabled;
                    instance.updateProperties();
                }
            }

            // If Sidekick isn't enabled, don't do anything else!
            if (!isEnabled)
                return; // ====>

            // On every boot, ensure that the Sidekick subdirectory exists and is clean:
            FileWorker.cleanupCaptureSubdirectory();

            // Determine the build time from the app's BuildConfig file:
            //    public static final boolean DEBUG = Boolean.parseBoolean("true")
            //    public static final String LIBRARY_PACKAGE_NAME = "com.qualcomm.ftcrobotcontroller"
            //    public static final String BUILD_TYPE = "debug"
            //    public static final String APP_BUILD_TIME = "2026-02-04T19:00:19.296-0800"
            try {
                Class<?> buildConfigClass = Class.forName("com.qualcomm.ftcrobotcontroller.BuildConfig");
                Field appBuildTimeField = buildConfigClass.getDeclaredField("APP_BUILD_TIME");
                instance.appBuildTime = (String) appBuildTimeField.get(null);
            } catch (ClassNotFoundException | NoSuchFieldException | IllegalAccessException ignored) {}

            // If the app's build time has changed, reset the run number:
            if (!lastAppBuildTime.equals(instance.appBuildTime)) {
                instance.lastRunNumber = 0;
            }

            try {
                instance.server.start();
            } catch (IOException e) {
                // @@@ I hit this error after a spontaneous connection loss. FTC Dashboard's
                // connection worked, but not Sidekick's websocket connection, as a result of
                // this problem. Should retry multiple times? I didn't see the error message, though...
                logE("Couldn't start server, exception: " + e.getMessage());
            }

            // Now that we have the context, prepare our builder:
            instance.proxyBuilder = new ProxyBuilder(context);

            // Start the worker threads:
            instance.fileWorker = new FileWorker(instance.proxyBuilder);
            instance.sampleWorker = new SampleWorker();

            // Register listeners for opMode notifications that come back to this Sidekick class:
            eventLoop.getOpModeManager().registerListener(instance);

            // Remember the OpModeManager for later:
            instance.eventLoop = eventLoop;
            isInitialized = true;
        }
    }

    /// Register a special opMode to provide a UI means of enabling or disabling Sidekick on
    /// the next boot. This is the third method the system calls, after onCreateEventLoop.
    @OpModeRegistrar @SuppressWarnings("unused") public static void registerOpMode(OpModeManager manager) {
        logI("@@@ registerOpMode");
        // Don't enable the UI if the user has already called enable() or disable():
        if (apiRequestedState != ApiRequestedState.NOT_CALLED)
            return; // ====>

        manager.register(
                new OpModeMeta.Builder()
                        .setName("Enable/Disable Sidekick")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
                        .setGroup("Utility")
                        .build(),
                new LinearOpMode() {
                    @Override
                    public void runOpMode() {
                        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
                        String enabled = "<span style='background: #206040'>enabled</span>";
                        String disabled = "<span style='background: #823232'>disabled</span>";
                        String currentState = isEnabled ? enabled : disabled;
                        String nextState = nextBootEnabled ? enabled : disabled;
                        String toggleState = nextBootEnabled ? disabled : enabled;

                        String message = "<h2>Sidekick Setting</h2>" + "Sidekick is currently " +
                            currentState;
                        if (isEnabled != nextBootEnabled) {
                            message += ", but will be " + nextState + " on the next boot";
                        }
                        message += ".\n\nPress <font color='#05BD05'>▶</font> to toggle it to " +
                            toggleState + " on the next boot, or <font color='#C94F4F'>■</font> " +
                            "to leave it as is.";

                        telemetry.addLine(message);
                        telemetry.update();

                        waitForStart();
                        if (!isStopRequested()) {
                            // The user pressed start, so toggle the state:
                            nextBootEnabled = !nextBootEnabled;
                            instance.updateProperties();

                            telemetry.addLine("Sidekick will be " + toggleState + " on the next boot.");
                            telemetry.update();
                            while (!isStopRequested()) {
                                sleep(25);
                            }
                        }
                    }
                });
    }

    /// Initialization hook for the emulator; it's dynamically called via reflection.
    @SuppressWarnings("unused")
    public static Sidekick emulatorInitialization(BiFunction<Integer, Object, Object> callback, Context context, FtcEventLoop eventLoop) {
        onCreate(context);
        onCreateEventLoop(context, eventLoop);
        instance.emulatorCallback = callback;
        return instance;
    }

    /// One-time initialization we do in preparation for the first opMode.
    void oneTimeOpModeInitialization() {
        // Register a handler to caught unhandled exceptions:
        Thread.UncaughtExceptionHandler previous =
                Thread.getDefaultUncaughtExceptionHandler();

        logI("Previous unhandled handler: %s", previous != null ? previous.getClass().getName() : "null"); // @@@

        // Register a gamepad callback so that the system notifies us of gamepad updates.
        // Do this only on the first OpMode, not subsequent ones.
        NetworkConnectionHandler connectionHandler = NetworkConnectionHandler.getInstance();
        try {
            // Call connectionHandler.theRecvLoopCallback.push(gamepadCallback);
            Field recvLoopCallbackField = connectionHandler.getClass().getDeclaredField(
                    "theRecvLoopCallback");
            recvLoopCallbackField.setAccessible(true);
            Object recvLoopCallback = recvLoopCallbackField.get(connectionHandler);
            if (recvLoopCallback != null) {
                Class<?> recvLoopCallbackChainerClass = recvLoopCallback.getClass();
                Method pushMethod = recvLoopCallbackChainerClass.getDeclaredMethod(
                        "push", RecvLoopRunnable.RecvLoopCallback.class);
                pushMethod.setAccessible(true);
                pushMethod.invoke(recvLoopCallback, receiveLoopCallbacks);
                isGamepadHooked = true;
            }
        } catch (NoSuchFieldException | IllegalAccessException | NoSuchMethodException |
                 InvocationTargetException ignored) { }

        // Register default serializers:
        registerSerializer(
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0),
                "x=in, y=in, heading=rad",
                x -> new Double[] {
                        x.getX(DistanceUnit.INCH),
                        x.getY(DistanceUnit.INCH),
                        x.getHeading(AngleUnit.RADIANS)
                });
        registerSerializer(
                new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0),
                "yaw=rad, pitch=rad, roll=rad",
                x -> new Double[]{
                        x.getYaw(AngleUnit.RADIANS),
                        x.getPitch(AngleUnit.RADIANS),
                        x.getRoll(AngleUnit.RADIANS)
                });
    }

    /// Callback courtesy of [OpModeManagerNotifier.Notifications]: the user may have just pressed
    /// Init on their chosen opMode.
    @Override public void onOpModePreInit(OpMode opMode) {
        assert(isEnabled);
        String className = opMode.getClass().getName();
        logI("@@@ onOpModePreInit %s from thread %s", className, Thread.currentThread().getName());

        synchronized(sidekickLock) {
            if (expectedNotification != OpModeNotification.PRE_INIT) {
                logE("onOpModePreInit() called when expecting %s", expectedNotification);
                return; // ====>
            }
            expectedNotification = OpModeNotification.PRE_START; // Next expected notification

            // Let the SampleWorker thread know when any new opMode (including default) has started:
            opModeCount++;

            // Ignore notifications about the default opMode:
            isDefaultOpMode = (opMode instanceof OpModeManagerImpl.DefaultOpMode);
            if (isDefaultOpMode) {
                return; // ====>
            }

            // Increment the run number and save the updated properties:
            lastRunNumber++;
            updateProperties();

            // Now do some deferred one-time initialization when starting the first opMode:
            if (!oneTimeOpModeInitializationDone) {
                oneTimeOpModeInitialization();
            }
            oneTimeOpModeInitializationDone = true;

            // Before wrapping any objects, ensure that priming is complete:
            try {
                primingDoneSignal.await();
            } catch (InterruptedException ignored) {}

            // Initialize the capture:
            Capture.instance = new Capture(instance, opMode);
            Capture.instance.initializeCapture(eventLoop.getOpModeManager(), isGamepadHooked);
        }
    }

    /// Callback courtesy of [OpModeManagerNotifier.Notifications]: the user has just pressed
    /// Start on the active opMode.
    @Override public void onOpModePreStart(OpMode opMode) {
        assert(isEnabled);
        logI("@@@ onOpModePreStart from thread %s", Thread.currentThread().getName());
        synchronized(sidekickLock) {
            if (expectedNotification != OpModeNotification.PRE_START) {
                logE("@@@ onOpModePreStart() called when expected %s", expectedNotification);
                return; // ====>
            }
            expectedNotification = OpModeNotification.POST_STOP; // Next expected notification

            if ((!isDefaultOpMode) && (isEnabled)) {
                // Mark the start time for this capture:
                Capture.instance.markStartTime();
            }
        }
    }

    /// Callback courtesy of [OpModeManagerNotifier.Notifications]: the opMode is done, perhaps
    /// because the user has pressed Stop.
    @Override public void onOpModePostStop(OpMode opMode) {
        assert(isEnabled);
        logI("@@@ onOpModePostStop from thread %s", Thread.currentThread().getName());

        synchronized(sidekickLock) {
            // The system sometimes calls onOpModePostStop() multiple times; we only act on the
            // first call and ignore the others:
            if (expectedNotification == OpModeNotification.PRE_INIT) {
                logE("@@@ onOpModePostStop() called when expecting %s", expectedNotification);
                return; // ====>
            }
            expectedNotification = OpModeNotification.PRE_INIT; // Next expected notification

            if (!isDefaultOpMode) {
                Capture.instance.endCapture();
            }
        }
    }
}
