/// Logic for the Sidekick sampling thread.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import android.annotation.SuppressLint;
import android.util.Log;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.io.FileInputStream;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;
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
import java.util.function.Supplier;

/// Descriptor for enumerating supported realtime counter types.
class RealtimeDescriptor {
    String name; // Name of the counter
    String description; // Description of the counter
    public RealtimeDescriptor(String name, String description) {
        this.name = name;
        this.description = description;
    }
}

/// Values repository for realtime counters.
class Realtime {
    static final Object lock = new Object(); // Lock for the realtime repository
    static LinkedList<RealtimeDescriptor> supportedCounters = new LinkedList<>(); // Supported counters
    static HashMap<Socket, Set<String>> subscriptions = new HashMap<>(); // List of counters by socket
    static HashMap<String, List<Float>> counterValues = new HashMap<>(); // List of accumulated values for each counter

    /// Return a list of supported realtime counters.
    static LinkedList<RealtimeDescriptor> getSupportedCounters() {
        return supportedCounters;
    }

    /// Register a supported realtime counter. Thread safe. Can be called multiple times for
    /// the same counter.
    static void register(String counterName, String description) {
        synchronized(lock) {
            supportedCounters.add(new RealtimeDescriptor(counterName, description));
        }
    }

    /// Return true if the specified counter is supported.
    static boolean isRegistered(String counterName) {
        synchronized(lock) {
            for (RealtimeDescriptor descriptor : supportedCounters) {
                if (descriptor.name.equals(counterName)) {
                    return true;
                }
            }
            return false;
        }
    }

    /// Update the list of realtime counters subscribed by the specified socket.
    static void updateSubscription(Socket socket, String[] counterNames) {
        synchronized (lock) {
            // Add or remove this socket from the realtime subscriber list, as necessary:
            Sidekick.logI("starting updateSubscription"); // @@@
            if (counterNames.length == 0) {
                subscriptions.remove(socket);
            } else {
                subscriptions.put(socket, new HashSet<>(Arrays.asList(counterNames)));
            }

            Sidekick.logI("more updateSubscription"); // @@@
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
            Sidekick.logI("finished updateSubscription");
        }
    }

    /// Return the list of realtime counters subscribed by all sockets. Thread safe.
    static Set<String> getSubscribedNames() {
        synchronized (lock) {
            // Make a copy of the values for thread safety:
            return new HashSet<>(counterValues.keySet());
        }
    }

    /// Counter producers call this to notify us of new values. We record the value if a
    /// socket has subscribed to this particular counter. Thread safe.
    static void post(String counterName, double value) {
        assert(isRegistered(counterName));
        // Note that we don't need to acquire the lock: there is never thread contention for
        // the same counter/list, and Java won't crash if unsubscribing at the same time.
        List<Float> values = counterValues.get(counterName);
        if (values != null) {
            // A socket has subscribed to this counter, so record the value:
            values.add((float) value);
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

/// Sampler of various CPU performance metrics.
class CpuSampler {
    // Governors docs: // https://android.googlesource.com/kernel/common/+/a7827a2a60218b25f222b54f77ed38f57aebe08b/Documentation/cpu-freq/governors.txt
    enum Governor { PERFORMANCE, POWER_SAVE, USERSPACE, ON_DEMAND, CONSERVATIVE, INTERACTIVE, OFFLINE, OTHER }
    long previousTotal;
    long previousIdle;
    boolean first = true; // True if the very first sample

    CpuSampler() {
        Realtime.register("system_load", "System load (%)");
        Realtime.register("online_count", "Count of cores online");
        Realtime.register("governor", "CPU governor");
        Realtime.register("frequency", "CPU frequency (MHz)");
        Realtime.register("temperature", "CPU temperature (C)");
    }

    /// Record of the data.
    static class Record {
        int systemLoad; // Percentage utilization [0-100]
        int onlineCount; // Count of cores currently online
        Governor governor; // Cluster governor
        int frequencyMhz; // Cluster frequency, MHz
        int temperature; // Temperature, centi-degrees

        void post() {
            Realtime.post("system_load", systemLoad);
            Realtime.post("online_count", onlineCount);
            Realtime.post("governor", governor.ordinal());
            Realtime.post("frequency", frequencyMhz);
            Realtime.post("temperature", temperature / 100.0);
        }

        void put(ByteBuffer chunk) {
            chunk.put((byte) systemLoad);
            chunk.put((byte) onlineCount);
            chunk.put((byte) governor.ordinal());
            chunk.putShort((short) frequencyMhz);
            chunk.putShort((short) temperature);
        }
        static final int PUT_SIZE = 7;
    }

    /// Sample CPU performance.
    Record sample() {
        Record result = new Record();
        result.systemLoad = readSystemLoad();
        result.onlineCount = readOnlineCount();
        result.temperature = readTemperature();
        result.frequencyMhz = readInt("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") / 1000;

        String governorString = SampleWorker.readFirstLine("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
        CpuSampler.Governor governor = CpuSampler.Governor.OTHER;
        if (governorString != null) {
            governorString = governorString.toLowerCase();
            if (result.onlineCount == 0) {
                governor = CpuSampler.Governor.OFFLINE;
            } else if (governorString.contains("interactive")) {
                governor = CpuSampler.Governor.INTERACTIVE;
            } else if (governorString.contains("performance")) {
                governor = CpuSampler.Governor.PERFORMANCE;
            } else if (governorString.contains("powersave")) {
                governor = CpuSampler.Governor.POWER_SAVE;
            } else if (governorString.contains("userspace")) {
                governor = CpuSampler.Governor.USERSPACE;
            } else if (governorString.contains("ondemand")) {
                governor = CpuSampler.Governor.ON_DEMAND;
            } else if (governorString.contains("conservative")) {
                governor = CpuSampler.Governor.CONSERVATIVE;
            }
        }
        result.governor = governor;
        return result;
    }

    /// Compute system load percentage from jiffies, returning an integer [0-100].
    int readSystemLoad() {
        String line = SampleWorker.readFirstLine("/proc/stat");
        if (line == null) {
            return 0;
        }

        SplitParser splitParser = new SplitParser(line, 0);
        long user = splitParser.getValue(1);
        long nice = splitParser.getValue(2);
        long system = splitParser.getValue(3);
        long idleVal = splitParser.getValue(4);
        long ioWait = splitParser.getValue(5);
        long irq = splitParser.getValue(6);
        long softIrq = splitParser.getValue(7);
        long steal = splitParser.getValue(8);

        long idleJ = idleVal + ioWait;
        long nonIdle = user + nice + system + irq + softIrq + steal;
        long total = idleJ + nonIdle;
        long idle = idleVal + ioWait;

        if (first) {
            previousTotal = total;
            previousIdle = idle;
            first = false;
            return 0;
        }

        long deltaTotal = total - previousTotal;
        long deltaIdle  = idle - previousIdle;

        previousTotal = total;
        previousIdle = idle;

        if (deltaTotal <= 0)
            return 0;
        return (int)((1f - (float)deltaIdle / (float)deltaTotal) * 100f);
    }

    /// Read CPU online state.
    int readOnlineCount() {
        String s = SampleWorker.readFirstLine("/sys/devices/system/cpu/online");
        if (s == null)
            return 0; // The Control Hub always supports this file, but phones might not

        int count = 0;
        int i = 0;
        final int n = s.length();

        while (i < n) {
            // Parse first number:
            int start = 0;
            while (i < n && Character.isDigit(s.charAt(i))) {
                start = start * 10 + (s.charAt(i) - '0');
                i++;
            }

            int end = start;

            // Check for range: '-'
            if (i < n && s.charAt(i) == '-') {
                i++; // skip '-'
                end = 0;
                while (i < n && Character.isDigit(s.charAt(i))) {
                    end = end * 10 + (s.charAt(i) - '0');
                    i++;
                }
            }

            // Count CPUs in this segment:
            count += (end - start + 1);

            // Skip comma if present:
            if (i < n && s.charAt(i) == ',') {
                i++;
            }
        }

        return count;
    }

    /// Read CPU temperature in Celsius centi-degrees.
    int readTemperature() {
        for (int i = 0; i < 10; i++) {
            String s = SampleWorker.readFirstLine("/sys/class/thermal/thermal_zone" + i + "/temp");
            if (s != null) {
                return (int) SampleWorker.parseLong(s) / 10; // Milli-degrees to centi-degrees
            }
        }
        return 0; // not found
    }

    /// Open the file specified by `path` and read the first line as an integer.
    @SuppressWarnings("SameParameterValue")
    static int readInt(String path) {
        try {
            String s = SampleWorker.readFirstLine(path);
            return s == null ? 0 : (int) SampleWorker.parseLong(s.trim());
        } catch (Exception e) {
            return 0;
        }
    }
}

/// Sampler for storage and network bandwidth performance metrics.
class BandwidthSampler {
    long previousReadSectors = -1;
    long previousWriteSectors = -1;
    long previousRxBytes = -1;
    long previousTxBytes = -1;

    BandwidthSampler() {
        Realtime.register("storage_reads", "Storage reads (MB/s)");
        Realtime.register("storage_writes", "Storage writes (MB/s)");
        Realtime.register("wifi_reads", "Wi-Fi reads (MB/s)");
        Realtime.register("wifi_writes", "Wi-Fi writes (MB/s)");
    }

    /// Record of the data.
    static class Record {
        double storageReadMbps;
        double storageWriteMbps;
        double rxMbps;
        double txMbps;

        void post() {
            Realtime.post("storage_reads", storageReadMbps);
            Realtime.post("storage_writes", storageWriteMbps);
            Realtime.post("wifi_reads", rxMbps);
            Realtime.post("wifi_writes", txMbps);
        }

        void put(ByteBuffer chunk) {
            chunk.putFloat((float) storageReadMbps);
            chunk.putFloat((float) storageWriteMbps);
            chunk.putFloat((float) rxMbps);
            chunk.putFloat((float) txMbps);
        }
        static final int PUT_SIZE = 16;
    }

    /// Sample storage and network bandwidth.
    Record sample(double inverseDeltaT) {
        final int SECTOR_SIZE = 512;  // typical for eMMC
        Record result = new Record();

        // Read storage I/O statistics:
        String line = SampleWorker.readFirstLine("/sys/block/mmcblk0/stat");
        if (line != null) {
            SplitParser splitParser = new SplitParser(line, 0);
            long sectorsRead = splitParser.getValue(2);
            long sectorsWritten = splitParser.getValue(6);

            long storageReadDelta = sectorsRead - previousReadSectors;
            long storageWriteDelta = sectorsWritten - previousWriteSectors;

            previousReadSectors = sectorsRead;
            previousWriteSectors = sectorsWritten;

            result.storageReadMbps = (storageReadDelta * SECTOR_SIZE * inverseDeltaT) / 1_000_000.0;
            result.storageWriteMbps = (storageWriteDelta * SECTOR_SIZE * inverseDeltaT) / 1_000_000.0;
        }

        // Read Wi-Fi I/O statistics:
        line = SampleWorker.readFirstLine("/sys/class/net/wlan0/statistics/rx_bytes");
        if (line != null) {
            long rxBytes = SampleWorker.parseLong(line);
            long rxDelta = rxBytes - previousRxBytes;
            previousRxBytes = rxBytes;
            result.rxMbps = (rxDelta * inverseDeltaT) / 1_000_000.0;
        }
        line = SampleWorker.readFirstLine("/sys/class/net/wlan0/statistics/tx_bytes");
        if (line != null) {
            long txBytes = SampleWorker.parseLong(line);
            long txDelta = txBytes - previousTxBytes;
            previousTxBytes = txBytes;
            result.txMbps = (txDelta * inverseDeltaT) / 1_000_000.0;
        }
        return result;
    }
}

/// Sampler for memory and thread count metrics.
class MemoryAndThreadCountSampler {
    MemoryAndThreadCountSampler() {
        Realtime.register("vm_rss", "Vm Resident Set Size (GB)");
        Realtime.register("mem_available", "Memory available (GB)");
        Realtime.register("thread_count", "Thread count");
    }

    /// Record of the data.
    static class Record {
        long vmRSS; // Vm Resident Set Size (VmRSS), kB
        long memAvailable; // kB
        long threadCount;

        void post() {
            Realtime.post("vm_rss", vmRSS / 1024.0 / 1024.0);
            Realtime.post("mem_available", memAvailable / 1024.0 / 1024.0);
            Realtime.post("thread_count", threadCount);
        }

        void put(ByteBuffer chunk) {
            chunk.putInt((int) vmRSS);
            chunk.putInt((int) memAvailable);
            chunk.put((byte) threadCount);
        }
        static final int PUT_SIZE = 9;
    }

    /// Sample memory used and available and thread count.
    Record sample() {
        Record result = new Record();
        readVmRssAndThreadCount(result);
        readMemAvailable(result);
        return result;
    }

    /// Return the value in the string that follows the first occurrence of `valueName`.
    long getValue(String s, String valueName) {
        int i = s.indexOf(valueName);
        if (i == -1)
            return 0;
        i += valueName.length();
        while ((i < s.length()) && (!Character.isDigit(s.charAt(i))))
            i++;
        int j = i;
        while ((j < s.length()) && (Character.isDigit(s.charAt(j))))
            j++;
        return (i != j) ? Long.parseLong(s.substring(i, j)) : 0;
    }

    /// Read Vm Resident Set Size (VmRSS) in kB as well as the current thread count.
    void readVmRssAndThreadCount(Record record) {
        String status = SampleWorker.readAll("/proc/self/status");
        if (status == null)
            return; // ====>
        record.vmRSS = getValue(status, "VmRSS:");
        record.threadCount = getValue(status, "Threads:");
    }

    /// Read memory available in kB, which is the best indicator of system pressure.
    void readMemAvailable(Record record) {
        String memInfo = SampleWorker.readAll("/proc/meminfo");
        if (memInfo == null)
            return; // ====>
        long memFree = getValue(memInfo, "MemFree:");
        long cached = getValue(memInfo, "Cached:");
        long buffers = getValue(memInfo, "Buffers:");

        // Approximation for older kernels like the Control Hub without MemAvailable:
        record.memAvailable = memFree + cached + buffers;
    }
}

/// Sampler for Sidekick metrics.
class SidekickSampler {
    int previousBytesWritten = 0;

    /// Record of the data.
    static class Record {
        double writtenBytesPerSec; // Rate of additions to the archives
        int pendingBytes; // Count of bytes queued but not yet written

        void put(ByteBuffer chunk) {
            chunk.putFloat((float) writtenBytesPerSec);
            chunk.putInt(pendingBytes);
        }
        static final int PUT_SIZE = 8;
    }

    /// Sample Sidekick performance.
    Record sample(double inverseDeltaT) {
        Record record = new Record();
        record.pendingBytes = FileWorker.bytesQueued - FileWorker.bytesWritten;
        record.writtenBytesPerSec
                = Math.max(0, (FileWorker.bytesWritten - previousBytesWritten) * inverseDeltaT);
        previousBytesWritten = FileWorker.bytesWritten;
        return record;
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
                Capture.instance.error(MinorError.FTC_DASHBOARD_FIELD_OVERLAY);
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

/// Class to manage sampling of realtime counters where the sensor reads are slow and
/// injected into the capture.
class InjectedSampler {
    // Name of the goBilda Floodgate sensor for measuring current draw of the entire robot:
    final static String FLOODGATE_SENSOR = "floodgateSensor";

    /// Class to track active counters.
    static class Counter {
        String name; // Counter name
        Supplier<Double> supplier; // Supplier of counter value
        Counter(String name, Supplier<Double> supplier) {
            this.name = name;
            this.supplier = supplier;
        }
    }

    ArrayList<Counter> potentialCounters = new ArrayList<>(); // All potential counters
    ArrayList<Counter> activeCounters = new ArrayList<>(); // Active counters
    int counterIndex; // We round-robin through the counters; this is the current counter index

    /// Sample the next counter and record its value. May get called before [#prepareAndRegister].
    void sample() {
        // Wrap the counter index and check for a new active counter:
        if (counterIndex >= activeCounters.size()) {
            counterIndex = 0;
            Set<String> activeCounterNames = Realtime.getSubscribedNames();
            activeCounters.clear();
            for (Counter counter: potentialCounters) {
                if (activeCounterNames.contains(counter.name)) {
                    activeCounters.add(counter);
                }
            }
        }

        if (!activeCounters.isEmpty()) {
            // Sample the counter and advance for the next call. Note that there's no need
            // to mark the device calls as 'bonus' - the app recognizes this worker thread.
            Counter counter = activeCounters.get(counterIndex);
            Realtime.post(counter.name, counter.supplier.get());
            counterIndex++;
        }
    }

    /// Get a list of all wrapped hardware devices and their names of the specified type without
    /// recording the `hardwareMap.tryGet()` calls.
    static class Device<T extends HardwareDevice> {
        T device;
        String name;
        Device(T device, String name) { this.device = device; this.name = name; }
    }
    <T extends HardwareDevice> List<Device<T>> getDevices(HardwareMap hardwareMap, Class<? extends T> klass) {
        List<Device<T>> list = new LinkedList<>();
        Set<String> names = hardwareMap.getAllNames(LynxModule.class);
        for (String name: names) {
            T device = Capture.unwrap(hardwareMap).tryGet(klass, name); // Use original to avoid recording tryGet()
            if (device != null) {
                if (hardwareMap instanceof Capture.SkHardwareMap) { // Wrap the device if necessary
                    device = ((Capture.SkHardwareMap) hardwareMap).wrapDevice(device, name);
                }
                list.add(new Device<>(device, name));
            }
        }
        return list;
    }

    /// Prepare and register counters for sampling. Should be called every time there's a new
    /// opMode. `hardwareMap` will be [Capture.SkHardwareMap] if a user opMode, [HardwareMap]
    /// if the default system opMode.
    void prepareAndRegister(HardwareMap hardwareMap) {
        // Reset in preparation for gathering values:
        counterIndex = 0;
        potentialCounters = new ArrayList<>();
        String name;

        for (Device<DcMotorEx> motor: getDevices(hardwareMap, DcMotorEx.class)) {
            name = "motor." + motor.name + ".getCurrent";
            potentialCounters.add(new Counter(name, () -> motor.device.getCurrent(CurrentUnit.AMPS)));
            Realtime.register(name, motor.name + " current (A)");
        }

        for (Device<LynxModule> module: getDevices(hardwareMap, LynxModule.class)) {
            name = "module." + module.name + ".getCurrent";
            potentialCounters.add(new Counter(name, () -> module.device.getCurrent(CurrentUnit.AMPS)));
            Realtime.register(name, module.name + "current (A)");

            name = "module." + module.name + ".getGpioBusCurrent";
            potentialCounters.add(new Counter(name, () -> module.device.getGpioBusCurrent(CurrentUnit.AMPS)));
            Realtime.register(name, module.name + "%s GPIO bus current (A)");

            name = "module." + module.name + ".getI2cBusCurrent";
            potentialCounters.add(new Counter(name, () -> module.device.getI2cBusCurrent(CurrentUnit.AMPS)));
            Realtime.register(name, module.name + "%s I2C bus current (A)");

            name = "module." + module.name + ".getInputVoltage";
            potentialCounters.add(new Counter(name, () -> module.device.getInputVoltage(VoltageUnit.VOLTS)));
            Realtime.register(name, module.name + "%s input voltage (V)");

            name = "module." + module.name + ".getAuxiliaryVoltage";
            potentialCounters.add(new Counter(name, () -> module.device.getAuxiliaryVoltage(VoltageUnit.VOLTS)));
            Realtime.register(name, module.name + "%s auxiliary (5V) voltage (V)");

            name = "module." + module.name + ".getTemperature";
            potentialCounters.add(new Counter(name, () -> module.device.getTemperature(TempUnit.CELSIUS)));
            Realtime.register(name, module.name + "%s temperature (C)");
        }

        for (Device<Limelight3A> limelight: getDevices(hardwareMap, Limelight3A.class)) {
            // If multiple Limelight counters are active, it would be more optimal to aggregate
            // their queries into a single getStatus() call.
            name = "limelight." + limelight.name + ".getTemp";
            potentialCounters.add(new Counter(name, () -> limelight.device.getStatus().getTemp()));
            Realtime.register(name, limelight.name + "temperature (C)");

            name = "limelight." + limelight.name + ".getFps";
            potentialCounters.add(new Counter(name, () -> limelight.device.getStatus().getFps()));
            Realtime.register(name, limelight.name + "processing FPS");

            name = "limelight." + limelight.name + ".getCpu";
            potentialCounters.add(new Counter(name, () -> limelight.device.getStatus().getCpu()));
            Realtime.register(name, limelight.name + "CPU utilization (%)");
        }

        for (Device<AnalogInput> analogInput: getDevices(hardwareMap, AnalogInput.class)) {
            if (!analogInput.name.equals(FLOODGATE_SENSOR)) {
                name = "analogInput." + analogInput.name + ".getVoltage";
                // 3.3V on the sensor maps to 80A on V2 (we don't bother with V1's 60A because
                // that has been recalled):
                potentialCounters.add(new Counter(name, () -> analogInput.device.getVoltage() / 3.3 * 80.0));
                Realtime.register(name, "Robot current (from Floodgate) (A)");
            }
        }
    }
}

/// Class to sample threads and process CPU usage.
class ThreadSampler {
    /// Get the current jiffy count for the process.
    static int getProcessJiffies() {
        return readJiffies("/proc/self/stat");
    }

    /// Get the current jiffy count for the specified Linux TID. Thread 0 doesn't exist.
    static int getThreadJiffies(int tid) {
        return (tid == 0) ? -1 : readJiffies("/proc/self/task/" + tid + "/stat");
    }

    /// Read the pseudo-file to get the current jiffy count. Return -1 if thread doesn't exist.
    /// A jiffy is typically 10ms.
    private static int readJiffies(String path) {
        String contents = SampleWorker.readAll(path);
        if (contents == null)
            return -1;
        int index = contents.lastIndexOf(')'); // Find the end of the name; it may have spaces
        if (index < 0)
            return 0;
        SplitParser splitParser = new SplitParser(contents, index + 2);
        long userTime = splitParser.getValue(11);
        long systemTime = splitParser.getValue(12);

        return (int) (userTime + systemTime);
    }
}

/// Helper to parse space-delimited numbers in a string. This is a faster alternative
/// to String.split() and Long.parseLong().
class SplitParser {
    String string; // String to parse
    int offset; // Current index in the string
    int index; // Most recent field index

    SplitParser(String string, int offset) {
        this.string = string;
        this.offset = offset;
    }

    /// Get the value of the specified field. The fields must always be retrieved in order.
    long getValue(int targetIndex) { // Zero-based count
        int startOffset;
        do {
            // Skip leading spaces:
            while (string.charAt(offset) == ' ') {
                offset++;
                if (offset >= string.length())
                    return 0; // ====>
            }

            // Find the end of the field:
            startOffset = offset;
            while ((offset < string.length()) && (string.charAt(offset) != ' ')) {
                offset++;
            }
            index++;
        } while (index <= targetIndex); // Exit loop with index == targetIndex + 1 for next iteration

        long value = 0;
        for (int i = startOffset; i < offset; i++) {
            char c = string.charAt(i);
            if (!Character.isDigit(c)) {
                return 0; // ====>
            } else {
                value = value * 10 + (c - '0');
            }
        }
        return value;
    }
}

/// Sidekick's thread for periodic sampling work.
class SampleWorker {
    static final int WORK_QUANTUM = 50; // Check for work every 50ms
    static final int REALTIME_UPDATE_PERIOD = 250; // Send realtime update to PC every quarter second

    int opModeCount; // Track the number of opModes we've processed
    OpMode opMode; // The current opMode
    String globalErrorMessage = ""; /// Last result from [RobotLog#getGlobalErrorMsg]
    String globalWarningMessage = ""; /// Last result from [RobotLog#getGlobalWarningMessage]
    boolean first = true; /// True if the first call to [#doWork]
    double lastSystemSampleTime = 0; /// Time of last [#systemSample] call, in seconds
    DashboardSampler dashboardSampler = new DashboardSampler(); // Sample FTC Dashboard
    CpuSampler cpuSampler = new CpuSampler(); // CPU stats
    BandwidthSampler bandwidthSampler = new BandwidthSampler(); // Storage stats
    MemoryAndThreadCountSampler memoryAndThreadCountSampler = new MemoryAndThreadCountSampler();
    SidekickSampler sidekickSampler = new SidekickSampler(); // Sidekick stats
    InjectedSampler injectedSampler = new InjectedSampler(); // Counters

    // @@@ Should all the periods start off staggered?
    PeriodTracker systemSamplePeriod = new PeriodTracker(250); // Sample system counters every quarter second
    PeriodTracker dashboardSamplePeriod = new PeriodTracker(50); // Sample FTC Dashboard updates every 50ms
    PeriodTracker injectedSamplePeriod = new PeriodTracker(50); // Inject a counter every 50ms
    PeriodTracker realtimeUpdatePeriod = new PeriodTracker(REALTIME_UPDATE_PERIOD); // Updates to PC
    PeriodTracker broadcastBatteryVoltagePeriod = new PeriodTracker(1111); // Broadcast voltage every second-ish
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
            Sidekick.instance.registerThreadStart(Sidekick.SAMPLE_WORKER_THREAD_NAME);
            first = false;
        }
        if (opModeCount != Sidekick.opModeCount) {
            // Whenever there's a new opMode, we query the hardware map for new hardware objects
            // and register counters for real-time sampling:
            opModeCount = Sidekick.opModeCount;
            opMode = Sidekick.instance.eventLoop.getOpModeManager().getActiveOpMode();
            if (opMode != null) {
                injectedSampler.prepareAndRegister(opMode.hardwareMap); // Either HardwareMap or SkHardwareMap
            } else {
                Sidekick.logE("SampleWorker: couldn't acquire HardwareMap");
            }
        }

        // To minimize contention, we do NOT acquire the Sidekick lock here. Rather,
        // once data has been collected to record into the capture, THAT'S when the lock
        // is acquired and the capture is checked.
        Capture capture = Capture.instance;
        long currentNanoTime = System.nanoTime();
        if (systemSamplePeriod.isDue(currentNanoTime)) {
            systemSample(capture, currentNanoTime); // Robust to null captures
        }
        if (injectedSamplePeriod.isDue(currentNanoTime)) {
            injectedSampler.sample();
        }
        if (realtimeUpdatePeriod.isDue(currentNanoTime)) {
            Realtime.push();
        }
        if (broadcastBatteryVoltagePeriod.isDue(currentNanoTime)) {
            // If the system's default OpMode is currently running, we call getVoltage()
            // ourselves and save the result; when a user's OpMode is running, we piggyback
            // off the voltage sensor reads that the system regularly does as well as
            // explicit reads by the user's libraries or code.
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
        }
    }

    /// Sample performance across system components: CPU, storage, memory, Sidekick.
    void systemSample(@Nullable Capture capture, long currentNanoTime) {
        double startTime = currentNanoTime * 1e-9; // Time in seconds
        double inverseDeltaT = (lastSystemSampleTime != 0) ? 1.0 / (startTime - lastSystemSampleTime) : 0.0;
        lastSystemSampleTime = startTime;

        CpuSampler.Record cpuRecord = cpuSampler.sample();
        BandwidthSampler.Record storageRecord = bandwidthSampler.sample(inverseDeltaT);
        MemoryAndThreadCountSampler.Record memoryRecord = memoryAndThreadCountSampler.sample();
        SidekickSampler.Record sidekickRecord = sidekickSampler.sample(inverseDeltaT);

        short acquisitionTime = (short) ((System.nanoTime() * 1e-9 - startTime) * 1e6); // Microseconds

        // Record samples into the capture:
        if ((capture != null) && (!capture.isEnded)) {
            capture.recordSamples(cpuRecord, storageRecord, memoryRecord, sidekickRecord, acquisitionTime);
        }

        // Post the sample values for real-time display in the app:
        cpuRecord.post();
        storageRecord.post();
        memoryRecord.post();
    }

    /// Helper to open the file specified by `path` and read the first line as a string.
    static String readFirstLine(String path) {
        FileInputStream in = null;
        try {
            byte[] buffer = new byte[128]; // 128 bytes is plenty for one line
            in = new FileInputStream(path);
            int bytesRead = in.read(buffer);

            // Find first newline if it exists, otherwise return the whole buffer. The buffer
            // is ASCII so converting to a string is easy-peasy:
            int end = 0;
            while ((end < bytesRead) && (buffer[end] != '\n')) {
                end++;
            }
            return new String(buffer, 0, end);
        } catch (Exception e) {
            return null; // ====>
        } finally {
            // Be sure to always close the file!
            if (in != null) try { in.close(); } catch (Exception ignored) {}
        }
    }

    /// Helper to open the file specified by `path` and read it entirely as a string.
    static String readAll(String path) {
        FileInputStream in = null;
        try {
            byte[] buffer = new byte[2048]; // System pseudo files are tiny; 2 kB is plenty
            in = new FileInputStream(path);
            int bytesRead = in.read(buffer);
            return new String(buffer, 0, bytesRead);
        } catch (Exception e) {
            return null; // ====>
        } finally {
            // Be sure to always close the file!
            if (in != null) try { in.close(); } catch (Exception ignored) {}
        }
    }

    /// Helper to parse a string as a long.
    static long parseLong(String s) {
        try {
            return Long.parseLong(s);
        } catch (Exception e) {
            return 0;
        }
    }
}
