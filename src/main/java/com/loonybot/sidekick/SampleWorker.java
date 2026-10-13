/// Logic for the Sidekick sampling thread.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import android.annotation.SuppressLint;
import android.util.Log;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
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

/// Values repository for realtime counters. Non-instanced and only has static methods.
/// This is the interface between the Sockets and Sampling logic.
class Realtime {
    static final Object lock = new Object(); // Lock for the realtime repository
    static List<CounterDescriptor> supportedCounters = new ArrayList<>(); // List of supported realtime counters>
    static HashMap<Socket, Set<String>> subscriptions = new HashMap<>(); // List of subscribed counter IDs by socket
    static HashMap<String, List<Float>> counterValues = new HashMap<>(); // List of accumulated values for each counter
    static int updateUniqueness; // Uniqueness incremented every time a subscription changes
    static int queryUniqueness; // If this uniqueness is different from updateUniqueness, there's a new subscription

    /// Return a list of supported realtime counters. Empty if the counter tracker is not
    /// initialized. Thread safe.
    static List<CounterDescriptor> getSupportedCounters() {
        return Realtime.supportedCounters;
    }

    static void setSupportedCounters(List<CounterDescriptor> supportedCounters) {
        Realtime.supportedCounters = supportedCounters;
        updateUniqueness++;
        forceActiveCounterUpdate();
    }

    /// Update the list of realtime counters subscribed by the specified socket.
    static void updateSubscription(Socket socket, String[] counterIds) {
        synchronized (lock) {
            // Add or remove this socket from the realtime subscriber list, as necessary:
            if (counterIds.length == 0) {
                subscriptions.remove(socket);
            } else {
                subscriptions.put(socket, new HashSet<>(Arrays.asList(counterIds)));
            }

            // Update the aggregated list of all realtime counters subscribed by all sockets:
            Set<String> allSocketCounters = new HashSet<>();
            for (Set<String> counters: subscriptions.values()) {
                allSocketCounters.addAll(counters);
            }

            // Add any newly subscribed counters:
            for (String counter: allSocketCounters) {
                if (!counterValues.containsKey(counter)) {
                    counterValues.put(counter, new LinkedList<>());
                }
            }

            // Remove any counters that are no longer subscribed:
            for (String counter: new LinkedList<>(counterValues.keySet())) {
                if (!allSocketCounters.contains(counter)) {
                    counterValues.remove(counter);
                }
            }

            // Mark that the list of counters has changed:
            updateUniqueness++;
        }
    }

    /// Get the list of realtime counters subscribed by all sockets. Thread safe.
    static Set<String> getSubscription() {
        synchronized (lock) {
            // Make a copy of the values for thread safety:
            return new HashSet<>(counterValues.keySet());
        }
    }

    /// Return true if the list of active counters needs to be updated. This might be
    /// because a socket asked for new counters, or because the user called Sk.setSamplers().
    static boolean queryActiveCounterUpdate() {
        synchronized (lock) {
            boolean needsUpdate = queryUniqueness != updateUniqueness;
            queryUniqueness = updateUniqueness;
            return needsUpdate;
        }
    }

    /// Mark that the list of counters has changed. Thread safe.
    static void forceActiveCounterUpdate() {
        synchronized (lock) {
            updateUniqueness++;
        }
    }

    /// Append the new counter values to the queued list that is waiting to get pushed to the
    /// sockets. Thread safe.
    static void queue(List<CounterValue> newValues) {
        for (CounterValue newValue : newValues) {
            // Note that we don't need to acquire the lock: there is never thread contention for
            // the same counter/list, and Java won't crash if unsubscribing at the same time.
            List<Float> values = counterValues.get(newValue.id);
            if (values != null) {
                // A socket has subscribed to this counter, so record the value:
                values.add((float) newValue.value); // Cast from double
            }
        }
    }

    /// Push the accumulated realtime counters to all subscribed sockets.
    static void push() {
        synchronized(lock) {
            // Send the realtime values to subscribed sockets:
            for (Socket socket: subscriptions.keySet()) {
                Set<String> counters = subscriptions.get(socket);
                assert(counters != null);
                socket.pushRealtimeCounters(counters, counterValues);
            }

            // Clear the values in preparation for the next round:
            for (String counter: counterValues.keySet()) {
                List<Float> values = counterValues.get(counter);
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
                Capture.instance.minorError("FTC Dashboard field overlay");
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

    int opModeCount; // Track the number of opModes we've processed
    OpMode opMode; // The current opMode
    String globalErrorMessage = ""; /// Last result from [RobotLog#getGlobalErrorMsg]
    String globalWarningMessage = ""; /// Last result from [RobotLog#getGlobalWarningMessage]
    boolean first = true; /// True if the first call to [#doWork]
    DashboardSampler dashboardSampler = new DashboardSampler(); // Sample FTC Dashboard
    PeriodTracker counterSamplePeriod = new PeriodTracker(100); // Sample counters every 100ms
    PeriodTracker counterUpdatePeriod = new PeriodTracker(200); // Update the PC every fifth of a second
    PeriodTracker dashboardSamplePeriod = new PeriodTracker(50); // Sample FTC Dashboard updates every 50ms
    PeriodTracker broadcastBatteryVoltagePeriod = new PeriodTracker(500); // Broadcast voltage every half second
    PeriodTracker globalMessagePeriod = new PeriodTracker(500); // Check for global messages every 500ms
    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(runnable -> {
        Thread thread = new Thread(runnable);
        thread.setName(Sidekick.SAMPLE_WORKER_THREAD_NAME);
        return thread;
    });

    /// Constructor for SampleWorker that sets up the periodic task.
    @SuppressLint("DiscouragedApi") // Lint wants us to use scheduleWithFixedDelay
    SampleWorker() {
        scheduler.scheduleAtFixedRate(() -> {
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
        if (first) {
            // This can only be called by the current thread:
            Sidekick.instance.beginThread(Sidekick.SAMPLE_WORKER_THREAD_NAME);
            first = false;
        }
        if (opModeCount != Sidekick.opModeCount) {
            // Whenever there's a new opMode, we query the hardware map for new hardware objects
            // and register counters for real-time sampling:
            opModeCount = Sidekick.opModeCount;
            opMode = Sidekick.instance.eventLoop.getOpModeManager().getActiveOpMode();
            if (opMode != null) {
                // Inform the counter logic:
                Counters.instance.prepare(opMode.hardwareMap);

                // Inform other components of the results:
                Sidekick.instance.counters = Counters.instance.allCounters;
                Realtime.setSupportedCounters(Counters.instance.allCounters);

                // Now that the counters are prepared, record the values of some interesting ones
                // if we're doing a capture:
                if (Capture.instance != null) {
                    List<CounterValue> counters = Counters.instance.collectInteresting(false);
                    Capture.instance.recordCounterValues(counters);
                }
            } else {
                Sidekick.logE("SampleWorker: couldn't acquire HardwareMap");
            }
        }

        // To minimize contention, we do NOT acquire the Sidekick lock here. Rather,
        // once data has been collected to record into the capture, THAT'S when the lock
        // is acquired and the capture is checked.
        Capture capture = Capture.instance;
        long currentNanoTime = System.nanoTime();
        if (counterSamplePeriod.isDue(currentNanoTime)) {
            sampleCounters(capture); // Robust to null captures
        }
        if (counterUpdatePeriod.isDue(currentNanoTime)) {
            Realtime.push();
        }
        if (broadcastBatteryVoltagePeriod.isDue(currentNanoTime)) {
            // If the system's default OpMode is currently running, we call getVoltage()
            // ourselves and save the result; when a user's OpMode is running, we piggyback
            // off the voltage sensor reads that the system regularly does (as well as any
            // explicit reads by the user's libraries or code).
            if (opMode instanceof OpModeManagerImpl.DefaultOpMode) {
                List<VoltageSensor> sensors = opMode.hardwareMap.getAll(VoltageSensor.class);
                for (VoltageSensor sensor: sensors) {
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

    /// Sample performance counters.
    void sampleCounters(@Nullable Capture capture) {
        if (Realtime.queryActiveCounterUpdate()) {
            Counters.instance.updateActive(Realtime.getSubscription(), Sidekick.instance.userSetSamplers);
        }

        // Collect the counters:
        List<CounterValue> values = Counters.instance.queryActive();

        // Record system samples into the capture:
        if (capture != null) {
            capture.recordCounterValues(values);
        }

        // Queue all sample values for real-time display at the PC (it's okay that we provide
        // more IDs than were requested):
        Realtime.queue(values);
    }
}
