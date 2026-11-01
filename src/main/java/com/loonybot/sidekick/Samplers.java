/// Logic for the Sidekick samplers.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/// A collection of static helper methods for reading system files. Optimized for performance.
class SystemHelp {
    /// Helper to open the file specified by `path` and read the first line as an integer.
    @SuppressWarnings("SameParameterValue")
    static int readFileInteger(String path) {
        try {
            String s = readFileFirstLine(path);
            return s == null ? 0 : (int) parseLong(s.trim());
        } catch (Exception e) {
            return 0;
        }
    }

    /// Helper to open the file specified by `path` and read the first line as a string.
    static String readFileFirstLine(String path) {
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
    static String readFileAll(String path) {
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

    /// Helper to return the value in the string that follows the first occurrence of `valueName`.
    static long getValue(String s, String valueName) {
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

    /// Helper to parse space-delimited numbers in a string. This is a faster alternative
    /// to String.split() and Long.parseLong().
    static class SplitParser {
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
        String contents = readFileAll(path);
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

/// Descriptor for enumerating supported sampler types.
class SamplerDescriptor {
    String id; // Identifier of the sampler (e.g., "Control Hub.current", "system.temperature")
    int index; /// Index into the index into the [Samplers#allSamplers] list
    boolean isSystem; // True if this is a system sampler (as opposed to a device sampler)
    String description; // Description of the sampler
    String unit; // Unit (e.g., "C")
    boolean isInteger; // True if the value represents an integer
    Supplier<Double> query; // Query to get the value of the sampler
    public SamplerDescriptor(String id, int index, boolean isSystem, String description,
                             String unit, boolean isInteger, Supplier<Double> query) {
        this.id = id;
        this.index = index;
        this.isSystem = isSystem;
        this.description = description;
        this.unit = unit;
        this.isInteger = isInteger;
        this.query = query;
    }
}

/// Values repository for samplers.
class SamplerValue {
    String id; // Sampler identifier
    int index; /// Index of sampler in [Samplers#allSamplers] list
    double value; // The metadata says whether to interpret as an integer
    SamplerValue(String id, int index, double value) {
        this.id = id;
        this.index = index;
        this.value = value;
    }
}

/// Fundamental types of samplers.
enum SamplersType {
    DEVICE, // Queries a device API call (so the query call is automatically captured)
    SYSTEM, // Queries a system OS call (so the query calls are not automatically captured)
}

/// Track samplers and their state.
class Samplers {
    // These baseline samplers will always be collected:
    final static String[] REQUIRED_SAMPLER_IDS = {
            "system.frequency", "system.opModeUtilization",
    };

    // Our instanced state.
    final static Samplers instance = new Samplers(); // Singleton
    final static Object samplersLock = new Object(); // Samplers lock
    LinkedList<String> lynxModuleNames = new LinkedList<>(); // Name of all Lynx modules
    LinkedList<DeviceBundle<?>> deviceBundles = new LinkedList<>(); // All bundles of device samplers
    LinkedList<SystemBundle> systemBundles = new LinkedList<>(); // All bundles of system samplers
    LinkedList<SamplerDescriptor> allSamplers = new LinkedList<>(); // Collection of all samplers
    HashMap<String, SamplerDescriptor> samplersById = new HashMap<>(); // Covers all samplers
    HashMap<String, Supplier<Boolean>> setupsById = new HashMap<>(); // Covers samplers with setups
    HashMap<Supplier<Boolean>, List<SamplerDescriptor>> samplersBySetup = new HashMap<>();
    LinkedList<String> activeDeviceSamplers = new LinkedList<>(); // Actively subscribed device samplers
    LinkedList<String> activeSystemSamplers = new LinkedList<>(); // Actively subscribed system samplers
    int roundRobinIndex = 0; // Index into activeDeviceGroups for round-robin sampling
    float sampleAcquisitionTime = 0; // Time it took to query all samplers, ms

    /// A hardware device along with its friendly configuration name.
    static class DeviceDescriptor<T extends HardwareDevice> {
        Class<? extends T> klass; // Class of the device
        String name; // Friend name as shown in the robot configuration
        T device; // Hardware device
        DeviceDescriptor(Class<? extends T> klass, String name, T device) {
            this.klass = klass; this.name = name; this.device = device;
        }
    }

    /// A bundle contains samplers that are related and share common state and/or setup.
    abstract class SamplerBundle {
        SamplersType type; // Device vs. system sampler
        SamplerBundle(SamplersType type) { this.type = type; }

        /// Register a sampler during initialization for potential use.
        void register(String id, String description, String unit, boolean isInteger, Supplier<Double> query) {
            register(id, description, unit, isInteger, null, query);
        }

        /// Register a sampler during initialization for potential use.
        void register(String id, String description, String unit, boolean isInteger, Supplier<Boolean> setup, Supplier<Double> query) {
            SamplerDescriptor sampler = new SamplerDescriptor(id, allSamplers.size(),
                    (type == SamplersType.SYSTEM), description, unit, isInteger, query);
            if (setup != null) {
                samplersBySetup.computeIfAbsent(setup, k -> new LinkedList<>()).add(sampler);
                setupsById.put(id, setup);
            }
            samplersById.put(id, sampler);
            allSamplers.add(sampler);
        }

        /// Derived classes can implement this to renew their counters when a new op mode starts.
        /// NOTE: capture may be null!
        void renew(@Nullable Capture capture) {}
    }

    /// System samplers derive from this.
    class SystemBundle extends SamplerBundle {
        SystemBundle() { super(SamplersType.SYSTEM); }
    }

    /// Device samplers derive from this.
    abstract class DeviceBundle<T extends HardwareDevice> extends SamplerBundle {
        DeviceDescriptor<T> descriptor;
        DeviceBundle(DeviceDescriptor<T> descriptor) {
            super(SamplersType.DEVICE);
            this.descriptor = descriptor;
        }

        /// Get a new instance of the device when a new opMode is started.
        void renew(Capture capture, HardwareMap hardwareMap) {
            T device = hardwareMap.tryGet(descriptor.klass, descriptor.name);
            if (device != null) {
                descriptor = new DeviceDescriptor<>(descriptor.klass, descriptor.name, device);
            } else {
                Sidekick.logE("Couldn't renew hardware device");
            }
            super.renew(capture);
        }
    }

    /// Get a list of all hardware devices of the specified type and their names.
    <T extends HardwareDevice> List<DeviceDescriptor<T>> getDevices(HardwareMap hardwareMap, Class<? extends T> klass) {
        List<DeviceDescriptor<T>> list = new LinkedList<>();
        Set<String> names = hardwareMap.getAllNames(klass);
        for (String name: names) {
            T device = hardwareMap.tryGet(klass, name);
            if (device != null) {
                list.add(new DeviceDescriptor<>(klass, name, device));
            }
        }
        return list;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    /// Return samplers for Lynx modules.
    class LynxBundle extends DeviceBundle<LynxModule> {
        LynxBundle(DeviceDescriptor<LynxModule> initialDescriptor) {
            super(initialDescriptor);
            register(descriptor.name + ".current", "Hub current", "A", false,
                    () -> descriptor.device.getCurrent(CurrentUnit.AMPS));
            register(descriptor.name + ".gpioBusCurrent", "Hub GPIO bus current", "A", false,
                    () -> descriptor.device.getGpioBusCurrent(CurrentUnit.AMPS));
            register(descriptor.name + ".i2cBusCurrent", "Hub I2C bus current", "A", false,
                    () -> descriptor.device.getI2cBusCurrent(CurrentUnit.AMPS));
            register(descriptor.name + ".auxiliaryVoltage", "Hub auxiliary voltage (should be 5V)", "V", false,
                    () -> descriptor.device.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            register(descriptor.name + ".temperature", "Hub temperature", "C", false,
                    () -> descriptor.device.getTemperature(TempUnit.CELSIUS));
            // Deprecate getInputVoltage() in favor of VoltageSensor.
        }
    }

    /// Return samplers for the voltage sensor.
    class VoltageBundle extends DeviceBundle<VoltageSensor> {
        VoltageBundle(DeviceDescriptor<VoltageSensor> initialDescriptor) {
            super(initialDescriptor);
            register(descriptor.name + ".voltage", "Hub voltage", "V", false,
                    () -> descriptor.device.getVoltage());
        }
    }

    /// Return samplers for Limelight 3A.
    class LimelightBundle extends DeviceBundle<Limelight3A> {
        LLStatus status;
        LimelightBundle(DeviceDescriptor<Limelight3A> initialDescriptor) {
            super(initialDescriptor);
            register(descriptor.name + ".temperature", "Limelight temperature", "C", false,
                    this::setup, () -> status.getTemp());
            register(descriptor.name + ".frameTime", "Limelight frame time", "ms", false,
                    this::setup, () -> status.getFps() != 0 ? 1000.0 / status.getFps() : 0);
            register(descriptor.name + ".cpuUtilization", "Limelight CPU utilization", "%", false,
                    this::setup, () -> status.getCpu());
        }
        boolean setup() {
            status = descriptor.device.getStatus();
            return true;
        }
    }

    /// Sampler for the goBilda Floodgate sensor.
    class FloodgateBundle extends DeviceBundle<AnalogInput> {
        FloodgateBundle(DeviceDescriptor<AnalogInput> initialDescriptor) {
            super(initialDescriptor);
            // 3.3V on the sensor maps to 80A on V2 (we don't bother with V1's 60A because
            // those have been recalled):
            register(descriptor.name + ".current", "Current draw of main power rail", "A", false,
                    () -> descriptor.device.getVoltage() / 3.3 * 80.0);
        }
    }

    /// Sampler for DcMotorEx state.
    class MotorBundle extends DeviceBundle<DcMotorEx> {
        MotorBundle(DeviceDescriptor<DcMotorEx> initialDescriptor) {
            super(initialDescriptor);
            register(descriptor.name + ".current", "Motor current draw", "A", false,
                    () -> descriptor.device.getCurrent(CurrentUnit.AMPS));
            // Convert degrees/second to revolutions/minute.
            register(descriptor.name + ".rpm", "Motor RPM (if encoder connected)", "RPM", true,
                    () -> descriptor.device.getVelocity(AngleUnit.DEGREES) / 360.0 * 60.0);
        }
    }

    /// Return CPU core zero's frequency. The Control Hub seems to share the same clock
    /// for all CPU cores.
    class CpuFrequencyBundle extends SystemBundle {
        CpuFrequencyBundle() {
            register("system.frequency", "CPU frequency", "MHz", true,
                    () -> SystemHelp.readFileInteger("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") / 1000.0);
        }
    }

    /// Return the CPU governor mode. This appears to never change, in practice.
    class ScalingGovernorBundle extends SystemBundle {
        ScalingGovernorBundle() {
            register("system.governor", "CPU governor mode", "", true,
                    this::query);
        }
        double query() {
            String governorString = SystemHelp.readFileFirstLine("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
            int governor = -1;
            if (governorString != null) {
                governorString = governorString.toLowerCase();
                if (governorString.contains("interactive")) {
                    governor = 0;
                } else if (governorString.contains("performance")) {
                    governor = 1;
                } else if (governorString.contains("powersave")) {
                    governor = 2;
                } else if (governorString.contains("userspace")) {
                    governor = 3;
                } else if (governorString.contains("ondemand")) {
                    governor = 4;
                } else if (governorString.contains("conservative")) {
                    governor = 5;
                }
            }
            return governor;
        }
    }

    /// Compute system load percentage from jiffies, returning [0-100].
    class SystemUtilizationBundle extends SystemBundle {
        long previousTotal;
        long previousIdle;
        boolean first = true; // True if the very first sample
        double result;

        SystemUtilizationBundle() {
            register("system.cpuUtilization", "CPU utilization (all cores)", "%", false,
                    this::setup, () -> result);
        }
        boolean setup() {
            String line = SystemHelp.readFileFirstLine("/proc/stat");
            if (line == null)
                return false; // ====>

            SystemHelp.SplitParser splitParser = new SystemHelp.SplitParser(line, 0);
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
                return false; // ====>
            }

            long deltaTotal = total - previousTotal;
            long deltaIdle  = idle - previousIdle;

            previousTotal = total;
            previousIdle = idle;
            result = (deltaTotal <= 0) ? 0 : ((1f - (float)deltaIdle / (float)deltaTotal) * 100f);
            return true;
        }
    }

    /// Return the CPU utilization of the OpMode thread.
    class OpModeThreadBundle extends SystemBundle {
        Capture capture;
        int previousJiffies;
        long previousNanos;
        double result;

        OpModeThreadBundle() {
            register("system.opModeUtilization", "OpMode thread CPU utilization", "%", false,
                    this::setup, () -> result);
        }
        void renew(Capture capture) {
            this.capture = capture;
            if (capture != null) {
                this.previousJiffies = SystemHelp.getThreadJiffies(capture.opModeTid);
                this.previousNanos = System.nanoTime();
            }
        }
        boolean setup() {
            if ((capture != null) && (capture.opModeThreadDescriptor != null)) {
                int jiffies = SystemHelp.getThreadJiffies(capture.opModeTid);
                if (jiffies != -1) { // Thread exists and running when >= 0
                    long nanos = System.nanoTime();
                    result = 100.0 * (jiffies - previousJiffies) / Capture.nanosToJiffies(nanos - previousNanos);

                    // We receive notification that the OpMode is finished after the OpMode
                    // thread has already been terminated so we don't get a chance to
                    // read its final jiffy count. We make do with remembering its most
                    // recent sampled jiffy count instead. Note that we are safe doing this
                    // without acquiring a lock as there is no chance of contention:
                    ThreadDescriptor opModeThread = capture.opModeThreadDescriptor;
                    opModeThread.jiffyDuration = jiffies - opModeThread.jiffyStart;

                    // Prepare for the next call:
                    previousJiffies = jiffies;
                    previousNanos = nanos;
                    return true; // ====>
                }
            }
            return false;
        }
    }

    /// Read CPU online core count.
    class OnlineCoreBundle extends SystemBundle {
        double result;
        OnlineCoreBundle() {
            register("system.onlineCoreCount", "Count of cores online", "", true,
                    this::setup, () -> result);
        }
        boolean setup() {
            String s = SystemHelp.readFileFirstLine("/sys/devices/system/cpu/online");
            if (s == null)
                return false; // The Control Hub always supports this file, but phones might not

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
            result = count;
            return true;
        }
    }

    /// Read CPU temperature in Celsius Celsius.
    class CpuTemperatureBundle extends SystemBundle {
        double result;
        CpuTemperatureBundle() {
            register("system.temperature", "CPU temperature", "C", false,
                    this::setup, () -> result);
        }
        boolean setup() {
            for (int i = 0; i < 10; i++) {
                String s = SystemHelp.readFileFirstLine("/sys/class/thermal/thermal_zone" + i + "/temp");
                if (s != null) {
                    result = SystemHelp.parseLong(s) / 1000.0; // Milli-degrees C to degrees
                    return true; // ====>
                }
            }
            return false;
        }
    }

    /// Read storage I/O statistics.
    class StorageBandwidthBundle extends SystemBundle {
        final int SECTOR_SIZE = 512;  // Typical for eMMC
        long previousReadSectors = -1;
        long previousWriteSectors = -1;
        long previousMillis = System.currentTimeMillis();
        double readTotal;
        double writeTotal;
        double readTotalBias;
        double writeTotalBias;
        double readBandwidthResult;
        double writeBandwidthResult;

        StorageBandwidthBundle() {
            register("system.storageReadTotal", "Storage read total", "MB", false,
                    this::setup, () -> readTotal + readTotalBias);
            register("system.storageWriteTotal", "Storage write total", "MB", false,
                    this::setup, () -> writeTotal + writeTotalBias);
            register("system.storageReadRate", "Storage read bandwidth", "MB/s", false,
                    this::setup, () -> readBandwidthResult);
            register("system.storageWriteRate", "Storage write bandwidth", "MB/s", false,
                    this::setup, () -> writeBandwidthResult);
        }
        void renew(Capture capture) {
            // Reset the effective totals to zero at the start of every op mode:
            readTotalBias = -readTotal;
            writeTotalBias = -writeTotal;
        }
        boolean setup() {
            String line = SystemHelp.readFileFirstLine("/sys/block/mmcblk0/stat");
            if (line == null)
                return false;
            SystemHelp.SplitParser splitParser = new SystemHelp.SplitParser(line, 0);
            long sectorsRead = splitParser.getValue(2);
            long sectorsWritten = splitParser.getValue(6);
            double totalMultiplier = SECTOR_SIZE / 1_000_000.0; // Convert sectors to MBs

            long sectorsReadDelta = sectorsRead - previousReadSectors;
            long sectorsWrittenDelta = sectorsWritten - previousWriteSectors;
            previousReadSectors = sectorsRead;
            previousWriteSectors = sectorsWritten;

            long deltaMillis = System.currentTimeMillis() - previousMillis;
            previousMillis = System.currentTimeMillis();
            double inverseDeltaT = (deltaMillis != 0) ? 1000.0 / deltaMillis : 0; // Seconds
            double bandwidthMultiplier = totalMultiplier * inverseDeltaT; // Sectors to MB/s

            readTotal = sectorsRead * totalMultiplier;
            writeTotal = sectorsWritten * totalMultiplier;
            readBandwidthResult = sectorsReadDelta * bandwidthMultiplier;
            writeBandwidthResult = sectorsWrittenDelta * bandwidthMultiplier;
            return true;
        }
    }

    /// Read Wi-Fi I/O statistics.
    class WifiBandwidthBundle extends SystemBundle {
        long previousRxBytes = -1;
        long previousTxBytes = -1;
        long previousMillis = System.currentTimeMillis();
        double receiveTotal;
        double sendTotal;
        double receiveTotalBias;
        double sendTotalBias;
        double receiveRateResult;
        double sendRateResult;

        WifiBandwidthBundle() {
            register("system.wifiReceiveTotal", "Wi-Fi receive total", "MB", false,
                    this::setup, () -> receiveTotal + receiveTotalBias);
            register("system.wifiSendTotal", "Wi-Fi send total", "MB", false,
                    this::setup, () -> sendTotal + sendTotalBias);
            register("system.wifiReceiveRate", "Wi-Fi receive bandwidth", "MB/s", false,
                    this::setup, () -> receiveRateResult);
            register("system.wifiSendRate", "Wi-Fi send bandwidth", "MB/s", false,
                    this::setup, () -> sendRateResult);
        }
        void renew(Capture capture) {
            receiveTotalBias = -receiveTotal;
            sendTotalBias = -sendTotal;
        }
        boolean setup() {
            long deltaMillis = System.currentTimeMillis() - previousMillis;
            previousMillis = System.currentTimeMillis();
            double totalMultiplier = 1 / 1_000_000.0; // Bytes to MB
            double inverseDeltaT = (deltaMillis != 0) ? 1000.0 / deltaMillis : 0; // Seconds
            double rateMultiplier = totalMultiplier * inverseDeltaT; // Bytes to MB/s

            // Read Wi-Fi I/O statistics:
            String line = SystemHelp.readFileFirstLine("/sys/class/net/wlan0/statistics/rx_bytes");
            if (line != null) {
                long rxBytes = SystemHelp.parseLong(line);
                receiveTotal = rxBytes * totalMultiplier;
                long rxDelta = rxBytes - previousRxBytes;
                previousRxBytes = rxBytes;
                receiveRateResult = rxDelta * rateMultiplier;
            }
            line = SystemHelp.readFileFirstLine("/sys/class/net/wlan0/statistics/tx_bytes");
            if (line != null) {
                long txBytes = SystemHelp.parseLong(line);
                sendTotal = txBytes * totalMultiplier;
                long txDelta = txBytes - previousTxBytes;
                previousTxBytes = txBytes;
                sendRateResult = txDelta * rateMultiplier;
            }
            return true;
        }
    }

    /// Read available RAM.
    class MemAvailableBundle extends SystemBundle {
        double result;
        MemAvailableBundle() {
            register("system.memAvailable", "Available RAM (indicates system pressure)", "MB", false,
                    this::setup, () -> result);
        }
        boolean setup() {
            String memInfo = SystemHelp.readFileAll("/proc/meminfo");
            if (memInfo == null)
                return false; // ====>
            long memFree = SystemHelp.getValue(memInfo, "MemFree:"); // KB
            long cached = SystemHelp.getValue(memInfo, "Cached:"); // KB
            long buffers = SystemHelp.getValue(memInfo, "Buffers:"); // KB

            // Approximation for older kernels like the Control Hub without MemAvailable:
            result = (memFree + cached + buffers) / 1024.0; // KBs to MBs
            return true;
        }
    }

    /// Read VmRSS and thread count.
    class SelfStatusBundle extends SystemBundle {
        double vmrssResult;
        double threadCountResult;
        SelfStatusBundle() {
            register("system.vmrss", "VM Resident Set Size (VmRSS)", "MB", true,
                    this::setup, () -> vmrssResult);
            register("system.threadCount", "Thread count", "", true,
                    this::setup, () -> threadCountResult);

        }
        boolean setup() {
            String status = SystemHelp.readFileAll("/proc/self/status");
            if (status == null)
                return false; // ====>
            vmrssResult = SystemHelp.getValue(status, "VmRSS:") / 1024.0; // Reported in kB
            threadCountResult = SystemHelp.getValue(status, "Threads:");
            return true;
        }
    }

    /// Record the performance of Sidekick itself.
    class SidekickBundle extends SystemBundle {
        int previousBytesWritten;
        long previousMillis = System.currentTimeMillis();
        double pendingWritesResult;
        double writeRateResult;
        double sampleTimeResult;

        SidekickBundle() {
            register("sidekick.pendingWrites", "Sidekick write queue size", "MB", false,
                    this::setup, () -> pendingWritesResult);
            register("sidekick.writeRate", "Sidekick write bandwidth", "MB/s", false,
                    this::setup, () -> writeRateResult);
            register("sidekick.sampleTime", "Sidekick's time it takes to query samplers", "ms", false,
                    this::setup, () -> sampleTimeResult);
        }

        boolean setup() {
            long deltaMillis = System.currentTimeMillis() - previousMillis;
            previousMillis = System.currentTimeMillis();
            double inverseDeltaT = (deltaMillis != 0) ? 1000.0 / deltaMillis : 0; // Seconds

            pendingWritesResult = (FileWorker.bytesQueued - FileWorker.bytesWritten) / (1024.0 * 1024.4);
            writeRateResult = (FileWorker.bytesWritten - previousBytesWritten) * inverseDeltaT / (1024.0 * 1024.0);
            sampleTimeResult = sampleAcquisitionTime;

            previousBytesWritten = FileWorker.bytesWritten;
            return true;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // APIs

    /// This is called whenever a new OpMode is started. It informs us of the new HardwareMap
    /// and allows us to construct the sampler lists and initialize the hardware devices. The
    /// caller must hold appropriate locks. This should NOT be called while holding the Sidekick
    /// lock because it needs to create Byte Buddy proxies, which is extremely slow.
    List<SamplerDescriptor> prepare(Capture capture, HardwareMap unwrappedMap) {
        synchronized (samplersLock) {
            if (deviceBundles.isEmpty()) {
                // On the first prepare call, register the device samplers:
                for (DeviceDescriptor<LynxModule> lynx : getDevices(unwrappedMap, LynxModule.class)) {
                    lynxModuleNames.add(lynx.name);
                    deviceBundles.add(new LynxBundle(lynx));
                }
                for (DeviceDescriptor<VoltageSensor> voltage : getDevices(unwrappedMap, VoltageSensor.class)) {
                    deviceBundles.add(new VoltageBundle(voltage));
                }
                for (DeviceDescriptor<Limelight3A> limelight : getDevices(unwrappedMap, Limelight3A.class)) {
                    deviceBundles.add(new LimelightBundle(limelight));
                }
                for (DeviceDescriptor<AnalogInput> analog : getDevices(unwrappedMap, AnalogInput.class)) {
                    if (analog.name.equals(Sidekick.FLOODGATE_SENSOR)) {
                        deviceBundles.add(new FloodgateBundle(analog));
                    }
                }
                for (DeviceDescriptor<DcMotorEx> motor : getDevices(unwrappedMap, DcMotorEx.class)) {
                    deviceBundles.add(new MotorBundle(motor));
                }
                // Let the samplers know the capture instance if they want it:
                for (DeviceBundle<?> bundle : deviceBundles) {
                    bundle.renew(capture);
                }
            } else {
                // On subsequent prepare calls, recreate the device objects and let the samplers know:
                for (DeviceBundle<?> bundle : deviceBundles) {
                    bundle.renew(capture, unwrappedMap);
                }
            }
            if (systemBundles.isEmpty()) {
                // On the first prepare, register the system samplers:
                systemBundles.add(new CpuFrequencyBundle());
                systemBundles.add(new ScalingGovernorBundle());
                systemBundles.add(new SystemUtilizationBundle());
                systemBundles.add(new OpModeThreadBundle());
                systemBundles.add(new OnlineCoreBundle());
                systemBundles.add(new CpuTemperatureBundle());
                systemBundles.add(new StorageBandwidthBundle());
                systemBundles.add(new WifiBandwidthBundle());
                systemBundles.add(new MemAvailableBundle());
                systemBundles.add(new SelfStatusBundle());
                systemBundles.add(new SidekickBundle());
            }
            // Let the samplers know that a new op mode has started, along with the capture instance:
            for (SystemBundle bundle : systemBundles) {
                bundle.renew(capture);
            }
            /// Return a copy of the completed 'allSamplers' list:
            return new LinkedList<>(allSamplers);
        }
    }

    /// Query a specified set of samplers. Robust to invalid IDs.
    List<SamplerValue> query(List<String> ids) {
        // Create the list of setups and queries that we want to call, and call the setups:
        Set<Supplier<Boolean>> setups = new HashSet<>();
        Set<SamplerDescriptor> samplers = new HashSet<>(); // Set to protect against duplicates
        for (String id : ids) {
            Supplier<Boolean> setup = setupsById.get(id);
            if (setup == null) {
                // No setup routine so there's only one associated sampler (if the sampler is valid):
                SamplerDescriptor sampler = samplersById.get(id);
                if (sampler != null) {
                    samplers.add(samplersById.get(id));
                }
            } else {
                // If we haven't already called this setup routine, do so and add it to the set:
                if (!setups.contains(setup)) {
                    setups.add(setup);

                    // If the setup is successful, add all the associated queries to the set
                    // since their queries are basically free at this point:
                    if (setup.get()) {
                        List<SamplerDescriptor> associatedSamplers = samplersBySetup.get(setup);
                        if (associatedSamplers != null) {
                            samplers.addAll(associatedSamplers);
                        }
                    }
                }
            }
        }

        // We've called all the setups and accumulated all samplers to query so do the queries.
        // We don't bother marking as 'bonus' because the app special-cases everything from
        // the Sidekick Sampler Worker thread.
        List<SamplerValue> values = new ArrayList<>();
        for (SamplerDescriptor sampler : samplers) {
            values.add(new SamplerValue(sampler.id, sampler.index, sampler.query.get()));
        }
        return values;
    }

    /// Return the list of samplers that will be recorded regardless of what samplers are
    /// requested by connected clients.
    Set<String> getBaselineSamplerSet() {
        Set<String> ids = new HashSet<>(Arrays.asList(REQUIRED_SAMPLER_IDS));

        // If the user has called Sk.setSamplers() (even if to set an empty set), then
        // add those to the set. If the user has NOT called Sk.setSamplers(), our default
        // is to add the Floodgate sensor current if it exists, otherwise to add each hub's
        // current sensor.
        if (Sidekick.instance.userSetSamplers != null) {
            Collections.addAll(ids, Sidekick.instance.userSetSamplers);
        } else {
            if (samplersById.get(Sidekick.FLOODGATE_SENSOR + ".current") != null) {
                ids.add(Sidekick.FLOODGATE_SENSOR + ".current");
            } else {
                for (String lynxName : lynxModuleNames) {
                    ids.add(lynxName + ".current");
                }
            }
        }

        return ids;
    }

    /// Call this to update the list of active samplers in response to new subscription requests
    /// from connected clients or to new Sk.setSamplers() requests.
    void updateActiveSamplerList(Set<String> subscriptionIds) {
        synchronized (samplersLock) {
            // Make a union of subscription IDs, required IDs, and user-set IDs:
            Set<String> ids = new HashSet<>(subscriptionIds);
            ids.addAll(getBaselineSamplerSet());

            // Categorize the IDs into system vs. device samplers:
            activeDeviceSamplers.clear();
            activeSystemSamplers.clear();
            for (String id : ids) {
                SamplerDescriptor descriptor = samplersById.get(id);
                if (descriptor != null) {
                    if (descriptor.isSystem) {
                        activeSystemSamplers.add(id);
                    } else {
                        activeDeviceSamplers.add(id);
                    }
                }
            }
        }
    }

    /// Query from the active samplers. Called periodically (100 ms) from the sampler thread.
    List<SamplerValue> queryActive() {
        synchronized (samplersLock) {
            long startTime = System.nanoTime();
            List<SamplerValue> values = query(activeSystemSamplers);

            // Device samplers are uniformly slow, so we only do one every collection
            // period in a round-robin between devices. Because they are device queries,
            // they are also automatically recorded into the capture.
            if (!activeDeviceSamplers.isEmpty()) {
                roundRobinIndex = (roundRobinIndex + 1) % activeDeviceSamplers.size();
                String luckyDevice = activeDeviceSamplers.get(roundRobinIndex);
                values.addAll(query(Collections.singletonList(luckyDevice)));
            }

            // Acquisition time is one always one query delayed:
            sampleAcquisitionTime = (System.nanoTime() - startTime) * 1e-6f; // Milliseconds
            return values;
        }
    }

    /// Return the number of real time updates necessary before any device sampler is re-sampled.
    int getRefreshInterval() {
        return activeDeviceSamplers.size();
    }

    /// Collect values from the specified set of samplers in response to a [Sk#querySamplers]  call.
    List<SamplerValue> collectSet(String... ids) {
        synchronized (samplersLock) {
            return query(Arrays.asList(ids));
        }
    }

    /// Collect values from interesting device and system samplers at the beginning and end of
    /// a capture.
    List<SamplerValue> queryInterestingSamplers(@SuppressWarnings("unused") boolean isEnd) {
        // All system samplers that might be useful if sampled only at the start and end
        // of the capture:
        final String[] baselineInterestingSamplers = {
                "system.memAvailable", "system.vmrss", "system.threadCount",
                "system.storageReadTotal", "system.storageWriteTotal",
                "system.wifiReceiveTotal", "system.wifiSendTotal",
        };
        synchronized (samplersLock) {
            // Add everything from our 'interesting' list:
            LinkedList<String> ids = new LinkedList<>(Arrays.asList(baselineInterestingSamplers));

            // Add all available temperature samplers:
            for (SamplerDescriptor descriptor : allSamplers) {
                // Collect all temperature samplers:
                if (descriptor.id.contains("temperature")) {
                    ids.add(descriptor.id);
                }
            }
            return query(ids);
        }
    }
}
