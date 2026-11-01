/// Logic for the Sidekick sampling thread.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.LockSupport;

/// This class manages a worker thread that samples and records gamepad input. The thread is
/// long-lived and is notified whenever a new op mode starts.
///
/// Gamepad input is an essential part of a capture for doing postmortem analysis. I
/// considered various alternatives for hooking the data before finally settling on this
/// method.
///
/// * There is no apparent way to wrap Gamepad accesses.
/// * We can't hook [RecvLoopRunnable.RecvLoopCallback] like [EventLoopManager#gamepadEvent] does.
/// * We can't replace OpModeInternal.gamepad1 with a derived object that hooks [Gamepad#copy]
///   because that field gets regularly overwritten.
/// * We can't sample at a slow rate (e.g., 40 ms) because we don't know when the robot code
///   actually accessed the new data. It's too misleading to show new input coming in possibly
///   several control loops after it actually did.
///
/// If someone were to modify the SDK to support this scenario better, one approach would be
/// just two lines of code: add a public 'long' field to Gamepad that is the most recent copy()
/// time, and update it with nanoTime() on every copy() call. That would let Sidekick change to
/// a much slower sampling rate while precisely attributing gamepad input time.
class GamepadWorker {
    volatile OpMode opMode; // Currently running op mode; guaranteed non-null when the loop runs
    volatile Capture capture; // Current capture; guaranteed non-null when the loop runs
    Thread thread; // Our worker thread
    long timestamp1; // Timestamp of the most recent gamepad1 object
    long timestamp2; // Timestamp of the most recent gamepad2 object

    /// Register a new op mode. On the first invocation, start the worker thread.
    void initializeNewOpMode(OpMode opMode, Capture capture) {
        if (capture != null) {
            this.opMode = opMode;
            this.capture = capture; // Always keep this non-null so we can avoid locks

            if (thread == null) {
                thread = new Thread(this::loop);
                thread.setPriority(6); // Normal + 1
                thread.start();
            }
        }
    }

    /// Endless loop run by the dedicated gamepad worker thread.
    void loop() {
        Thread.currentThread().setName(Sidekick.GAMEPAD_WORKER_THREAD_NAME);
        Sidekick.instance.beginThread(Sidekick.GAMEPAD_WORKER_THREAD_NAME);

        while (!Thread.interrupted()) {
            LockSupport.parkNanos(5_000_000); // Sleep for at least 5 ms

            // Note that we don't take the overhead of thread synchronization when accessing
            // 'opMode' or 'capture'. As a result, they may be inconsistent between themselves
            // or stale - both of which are fine. Java guarantees that they at least always
            // reference valid objects.
            Gamepad gamepad1 = opMode.gamepad1;
            if ((gamepad1 != null) && (gamepad1.timestamp != timestamp1)) {
                timestamp1 = gamepad1.timestamp;
                capture.recordGamepad(gamepad1.toByteArray());
            }
            Gamepad gamepad2 = opMode.gamepad2;
            if ((gamepad2 != null) && (gamepad2.timestamp != timestamp2)) {
                timestamp2 = gamepad2.timestamp;
                capture.recordGamepad(gamepad2.toByteArray());
            }
        }
    }
}


/// Values repository for realtime samplers. Non-instanced and only has static methods.
/// This is the interface between the Sockets and Sampling logic.
class Realtime {
    static final Object realtimeLock = new Object(); // Lock for the realtime repository
    static List<SamplerDescriptor> supportedSamplers = new ArrayList<>(); // List of supported realtime samplers
    static List<String> baselineSamplerIds = new ArrayList<>(); // Identifies which of the supportedSamplers are baseline
    static HashMap<Socket, Set<String>> subscriptions = new HashMap<>(); // List of subscribed sampler IDs by socket
    static HashMap<String, List<Float>> samplerValues = new HashMap<>(); // List of accumulated values for each sampler
    static int updateUniqueness; // Uniqueness incremented every time a subscription changes
    static int queryUniqueness; // If this uniqueness is different from updateUniqueness, there's a new subscription

    /// Return a list of supported realtime samplers. Empty if the sampler tracker is not
    /// initialized. Thread safe.
    static List<SamplerDescriptor> getSupportedSamplers() {
        return Realtime.supportedSamplers;
    }

    /// Return a list of baseline samplers. Empty if the sampler tracker is not initialized.
    /// Thread safe.
    static List<String> getBaselineSamplerIds() {
        return Realtime.baselineSamplerIds;
    }

    /// Set the list of supported samplers.
    static void setSupportedSamplers(List<SamplerDescriptor> supportedSamplers, List<String> baselineIds) {
        Realtime.supportedSamplers = supportedSamplers;
        Realtime.baselineSamplerIds = baselineIds;
        updateUniqueness++;
        forceActiveSamplerUpdate();
    }

    /// Update the list of realtime samplers subscribed by the specified socket.
    static void updateSubscription(Socket socket, String[] samplerIds) {
        synchronized (realtimeLock) {
            // Add or remove this socket from the realtime subscriber list, as necessary:
            if (samplerIds.length == 0) {
                subscriptions.remove(socket);
            } else {
                subscriptions.put(socket, new HashSet<>(Arrays.asList(samplerIds)));
            }

            // Update the aggregated list of all realtime samplers subscribed by all sockets:
            Set<String> allSocketSamplers = new HashSet<>();
            for (Set<String> samplers: subscriptions.values()) {
                allSocketSamplers.addAll(samplers);
            }

            // Add any newly subscribed samplers:
            for (String sampler: allSocketSamplers) {
                if (!samplerValues.containsKey(sampler)) {
                    samplerValues.put(sampler, new LinkedList<>());
                }
            }

            // Remove any samplers that are no longer subscribed:
            for (String sampler: new LinkedList<>(samplerValues.keySet())) {
                if (!allSocketSamplers.contains(sampler)) {
                    samplerValues.remove(sampler);
                }
            }

            // Mark that the list of samplers has changed:
            updateUniqueness++;
        }
    }

    /// Get the union of realtime samplers subscribed by all sockets. Thread safe.
    static Set<String> getSubscription() {
        synchronized (realtimeLock) {
            // Make a copy of the values for thread safety:
            return new HashSet<>(samplerValues.keySet());
        }
    }

    /// Return true if the list of active samplers needs to be updated. This might be
    /// because a socket asked for new samplers, or because the user called Sk.setSamplers().
    static boolean activeSamplerListNeedsUpdating() {
        synchronized (realtimeLock) {
            boolean needsUpdate = queryUniqueness != updateUniqueness;
            queryUniqueness = updateUniqueness;
            return needsUpdate;
        }
    }

    /// Mark that the list of samplers has changed. Thread safe.
    static void forceActiveSamplerUpdate() {
        synchronized (realtimeLock) {
            updateUniqueness++;
        }
    }

    /// Append the new sampler values to the queued list that is waiting to get pushed to the
    /// sockets. Thread safe.
    static void queue(List<SamplerValue> newValues) {
        for (SamplerValue newValue : newValues) {
            // Note that we don't need to acquire the lock: there is never thread contention for
            // the same sampler/list, and Java won't crash if unsubscribing at the same time.
            List<Float> values = samplerValues.get(newValue.id);
            if (values != null) {
                // A socket has subscribed to this sampler, so record the value:
                values.add((float) newValue.value); // Cast from double
            }
        }
    }

    /// Push the accumulated realtime samplers to all subscribed sockets.
    static void push() {
        synchronized(realtimeLock) {
            // Send the realtime values to subscribed sockets:
            for (Socket socket: subscriptions.keySet()) {
                Set<String> samplers = subscriptions.get(socket);
                assert(samplers != null);
                socket.pushRealtimeSamplers(samplers, samplerValues);
            }

            // Clear the values in preparation for the next round:
            for (String sampler: samplerValues.keySet()) {
                List<Float> values = samplerValues.get(sampler);
                assert(values != null);
                values.clear();
            }
        }
    }
}

/// Simple tracker of a periodic interval. Decides when a task is due.
class PeriodTracker {
    int periodMs; // Target period, in milliseconds
    long lastInvocationNs; // Time of last invocation, in nanoseconds
    PeriodTracker(int period) {
        this.periodMs = period;
        this.lastInvocationNs = System.nanoTime();
    }

    /// True if the period has elapsed.
    boolean isDue(long currentNanoTime) {
        // Consider it good if within half a quantum:
        if ((currentNanoTime - lastInvocationNs) * 1e-6 >= (periodMs - SampleWorker.WORK_QUANTUM / 2.0)) {
            lastInvocationNs = currentNanoTime;
            return true;
        }
        return false;
    }
}

/// Sample FTC Dashboard for Canvases.
class DashboardSampler {
    final List<?> pendingTelemetryList = getPendingTelemetryList(); // If non-null, is FTC Dashboard's 'pendingTelemetry' queue
    Field telemetryPacketFieldOverlayField; // If non-null, is the 'field' field of Telemetry Packet
    int lastSnarfleIndex = -1; // pendingTelemetry index of last time we snarfled
    Object lastSnarfleTelemetryPacket; // TelemetryPacket from our last snarfle

    /// A reference to FTC Dashboard's pendingTelemetryList allows us to copy canvases.
    List<?> getPendingTelemetryList() {
        // Find FTC Dashboard's 'pendingTelemetry' queue so we can monitor Canvas updates:
        List<?> pendingTelemetry = null;
        Class<?> ftcDashboardClass = null;
        try {
            ftcDashboardClass = Class.forName("com.acmerobotics.dashboard.FtcDashboard");
        } catch (ClassNotFoundException ignored) {}
        if (ftcDashboardClass != null) { // Non-null if FTC Dashboard is installed
            try {
                // FtcDashboard instance = FtcDashboard.getInstance():
                Field appBuildTimeField = ftcDashboardClass.getDeclaredField("instance");
                appBuildTimeField.setAccessible(true);
                Object instance = appBuildTimeField.get(null);
                if (instance != null) {
                    // DashboardCore core = instance.core:
                    Field coreField = ftcDashboardClass.getDeclaredField("core");
                    coreField.setAccessible(true);
                    Object core = coreField.get(instance);
                    if (core != null) {
                        // List<TelemetryPacket> pendingTelemetry = core.pendingTelemetry:
                        Field pendingTelemetryField = core.getClass().getDeclaredField("pendingTelemetry");
                        pendingTelemetryField.setAccessible(true);
                        pendingTelemetry = (List<?>) pendingTelemetryField.get(core);

                        // Prepare to decode TelemetryPacket.fieldOverlay:
                        Class<?> telemetryPacketClass = Class.forName("com.acmerobotics.dashboard.telemetry.TelemetryPacket");
                        telemetryPacketFieldOverlayField = telemetryPacketClass.getDeclaredField("fieldOverlay");
                        telemetryPacketFieldOverlayField.setAccessible(true);
                    }
                }
            } catch (ClassNotFoundException | NoSuchFieldException | IllegalAccessException e) {
                // Disable FTC telemetry capture and note the error:
                pendingTelemetry = null;
                Sidekick.logE("Couldn't get FTC Dashboard field overlay");
            }
        }
        return pendingTelemetry;
    }

    /// Check to see if there are any new Dashboard canvases and if so, capture them.
    void sample(Capture capture) {
        if ((pendingTelemetryList != null) && (telemetryPacketFieldOverlayField != null)) {
            // Collect the list of telemetry packets while holding the pendingTelemetryList lock:
            List<Object> telemetryPackets = new LinkedList<>();
            synchronized (pendingTelemetryList) {
                int snarfleStart = 0;
                // The pendingTelemetryList may have grown since we last snarfled so start
                // with the first new one:
                if ((lastSnarfleIndex >= 0) &&
                        (lastSnarfleIndex < pendingTelemetryList.size()) &&
                        (lastSnarfleTelemetryPacket == pendingTelemetryList.get(lastSnarfleIndex))) {
                    snarfleStart = lastSnarfleIndex + 1;
                }
                for (int i = snarfleStart; i < pendingTelemetryList.size(); i++) {
                    telemetryPackets.add(pendingTelemetryList.get(i));
                }
                // Remember the last snarfle:
                lastSnarfleIndex = pendingTelemetryList.size() - 1;
                if (lastSnarfleIndex >= 0) {
                    lastSnarfleTelemetryPacket = pendingTelemetryList.get(lastSnarfleIndex);
                }
            }
            // Now that we no longer hold the lock, record the canvases from the telemetry
            // packets:
            for (Object telemetryPacket: telemetryPackets) {
                try {
                    Object canvas = telemetryPacketFieldOverlayField.get(telemetryPacket);
                    if (canvas != null) {
                        capture.recordInferredDashboardCanvas(canvas);
                    }
                } catch (IllegalAccessException ignored) {}
            }
        }
    }
}

/// Sidekick's thread for periodic sampling work. Long-lived and survives opMode terminations.
class SampleWorker {
    static final int WORK_QUANTUM = 50; // Check for work every 50ms
    static final Object sampleWorkerLock = new Object(); // Lock for the sample worker

    Capture capture; // If non-null, the capture being recorded
    LynxModule canaryModule; // A Lynx module to determine if it's okay to sample devices
    List<VoltageSensor> defaultOpModeVoltageSensors; // If non-null, voltage sensors to use when running default opMode
    String globalErrorMessage = ""; /// Last result from [RobotLog#getGlobalErrorMsg]
    String globalWarningMessage = ""; /// Last result from [RobotLog#getGlobalWarningMessage]
    boolean first = true; /// True if the first call to [#doWork]
    DashboardSampler dashboardSampler = new DashboardSampler(); // Sample FTC Dashboard
    PeriodTracker samplerSamplePeriod = new PeriodTracker(100); // Sample samplers every 100ms
    PeriodTracker samplerUpdatePeriod = new PeriodTracker(200); // Update the PC every fifth of a second
    PeriodTracker dashboardSamplePeriod = new PeriodTracker(50); // Sample FTC Dashboard updates every 50ms
    PeriodTracker broadcastBatteryVoltagePeriod = new PeriodTracker(500); // Broadcast voltage every half second
    PeriodTracker globalMessagePeriod = new PeriodTracker(500); // Check for global messages every 500ms
    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(runnable -> {
        Thread thread = new Thread(runnable);
        thread.setName(Sidekick.SAMPLE_WORKER_THREAD_NAME);
        return thread;
    });

    /// Constructor for SampleWorker that sets up the periodic task.
    SampleWorker() {
        scheduler.scheduleWithFixedDelay(() -> {
            try {
                doWork();
            } catch (Throwable t) { // Never let exceptions escape — they kill the task
                Sidekick.logE("SampleWorker exception: %s, stack: %s",
                        t.getMessage(), Log.getStackTraceString(t));
            }
        }, 0, WORK_QUANTUM, TimeUnit.MILLISECONDS);
    }

    /// The sample worker thread's primary loop. Called at a period of [#WORK_QUANTUM].
    private void doWork() {
        synchronized(sampleWorkerLock) {
            if (first) {
                // This can only be called by the current thread:
                Sidekick.instance.beginThread(Sidekick.SAMPLE_WORKER_THREAD_NAME);
                first = false;
            }

            long currentNanoTime = System.nanoTime();
            if (samplerSamplePeriod.isDue(currentNanoTime)) {
                querySamplers();
            }
            if (samplerUpdatePeriod.isDue(currentNanoTime)) {
                Realtime.push();
            }
            if (broadcastBatteryVoltagePeriod.isDue(currentNanoTime) && okayToSample()) {
                // If the system's default OpMode is currently running, we call getVoltage()
                // ourselves and save the result; when a user's OpMode is running, we piggyback
                // off the voltage sensor reads that the system regularly does (as well as any
                // explicit reads by the user's libraries or code).
                if (defaultOpModeVoltageSensors != null) {
                    for (VoltageSensor sensor : defaultOpModeVoltageSensors) {
                        Capture.saveBatteryVoltage(sensor.getVoltage());
                    }
                }
                Socket.broadcastBatteryVoltage();
            }
            // We require captures to be active for the following:
            if ((capture != null) && (!capture.isEnded)) {
                if (dashboardSamplePeriod.isDue(currentNanoTime)) {
                    dashboardSampler.sample(capture);
                }
                if (globalMessagePeriod.isDue(currentNanoTime)) {
                    if (RobotLog.hasGlobalErrorMsg()) {
                        String message = RobotLog.getGlobalErrorMsg();
                        if (!message.equals(globalErrorMessage)) {
                            globalErrorMessage = message;
                            capture.recordGlobalMessage(true, globalErrorMessage);
                        }
                    }
                    if (RobotLog.hasGlobalWarningMsg()) {
                        String message = RobotLog.getGlobalWarningMessage().message;
                        if (!message.equals(globalWarningMessage)) {
                            globalWarningMessage = message;
                            capture.recordGlobalMessage(false, globalWarningMessage);
                        }
                    }
                }
                // End the capture if the opMode has passed the 120-second limit for TeleOp:
                if ((capture.startNanoTime != capture.initNanoTime) &&
                        ((currentNanoTime - capture.startNanoTime) > Capture.OP_MODE_TIME_LIMIT_NANOS) &&
                        !(capture.isEnded)) {
                    capture.endCapture(Capture.LOGCAT_OP_MODE_TIME_LIMIT);
                }
            }
        }
    }

    /// Query the samplers.
    void querySamplers() {
        if (Realtime.activeSamplerListNeedsUpdating()) {
            Samplers.instance.updateActiveSamplerList(Realtime.getSubscription());
        }

        if (okayToSample()) {
            // Collect from the samplers:
            List<SamplerValue> values = Samplers.instance.queryActive();

            // Record system samples into the capture:
            if (capture != null) {
                capture.recordSamplerValues(values);
            }

            // Queue all sample values for real-time display at the PC (it's okay that we provide
            // more IDs than were requested):
            Realtime.queue(values);
        }
    }

    /// Return 'true' if it's okay to use a hardware device to query a sampler. A scenario where
    /// it's not permissible to query a sampler is when exiting from 'Configure Devices'. Our
    /// usage isn't atomic with respect to the system, but that's okay as the system device
    /// calls themselves do their own check. However, our check is useful for avoiding Logcat
    /// spew for 99.99% of the time.
    boolean okayToSample() {
        return (canaryModule != null) && (canaryModule.isOpen());
    }

    /// Lazily initialize for a new opMode. 'capture' is null when running a default opMode.
    /// Do NOT call this while holding the Sidekick lock as that can create deadlocks - we
    /// often need to call capture code while holding the sampler worker lock.
    void initializeNewOpMode(HardwareMap unwrappedMap, Capture newCapture) {
        assert(!Thread.holdsLock(Sidekick.sidekickLock));
        synchronized(sampleWorkerLock) {
            capture = newCapture;
            Samplers samplers = Samplers.instance;

            // Get a new Lynx module using the new HardwareMap:
            canaryModule = unwrappedMap.getAll(LynxModule.class).get(0);

            // Prepare the sampler logic. These provide copied lists:
            List<SamplerDescriptor> allSamplers = samplers.prepare(capture, unwrappedMap);
            Set<String> baselineIds = samplers.getBaselineSamplerSet();

            // Inform other components of the results of the preparation:
            Realtime.setSupportedSamplers(allSamplers, new ArrayList<>(baselineIds));

            if (capture == null) {
                // When the default opMode is running, we can't rely on the hooked SkVoltageSensor
                // to monitor the voltage so hook them directly:
                defaultOpModeVoltageSensors = unwrappedMap.getAll(VoltageSensor.class);
            } else {
                defaultOpModeVoltageSensors = null;
                capture.allSamplers = allSamplers;

                // Record some interesting sampler values to start the capture:
                capture.markBonus(true);
                capture.recordSamplerValues(samplers.queryInterestingSamplers(false));
                capture.markBonus(false);
            }
        }
    }
}
