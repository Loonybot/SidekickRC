/// Sidekick capture logic.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import static com.loonybot.sidekick.Sidekick.MIN_STORAGE_MBS;
import static java.lang.System.nanoTime;

import android.os.StatFs;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

//import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.inspection.InspectionState;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.io.StringWriter;
import java.lang.annotation.Annotation;
import java.lang.annotation.Repeatable;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.IntBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.Charset;
import java.nio.charset.CharsetEncoder;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

/// The sidekick file is created almost entirely by continuous in-order appending. However, when
/// reading the file, parts of it have to be processed out-of-order from the writes. Sections are
/// partitions of the file that are indexed in the file header for this purpose.
@SuppressWarnings({"unused"})
class Section {
    // Constants describing the various sections. They won't be recorded in this order,
    // but *will* be uploaded in this order.
    static final byte METADATA           =  0; // String tokens, record descriptors, manifest
    static final byte PII                =  1; // Privacy-sensitive data, if any
    static final byte ARCHIVE            =  2; // Archive records, usually the biggest section
    static final byte LOGCAT             =  3; // Logcat section, if any
    static final byte RESERVED4          =  4;
    static final byte RESERVED3          =  5;
    static final byte RESERVED2          =  6;
    static final byte RESERVED1          =  7;
    // ---------------------------------------
    static final byte COUNT              =  8; // Reserve for this many sections in file header

    // Actual section record:
    int offset; // Offset from file start to the section, in bytes
    int size; // Size of the section, in bytes

    Section(int offset, int size) {
        this.offset = offset;
        this.size = size;
    }
}

/// Structure for the capture's header information.
class Header {
    final static int SIZE = 256; // Byte size of the file header

    // Raw contents:
    byte[] bytes = new byte[SIZE]; /// Raw byte contents of the header
    int magic; /// Unique signature for all Sidekick files; value is [Signature#FILE_HEADER]
    int libraryVersion; /// Library version number that created this file
    int requiredAppVersion; /// Minimum app version required to read this file
    int majorErrors; /// Bitmask of major errors; [MajorError] indicates the bit
    int minorErrors; /// Bitmask of minor errors; [MinorError] indicates the bit
    long startUnixTimeMs; /// Unix time when the capture started, in milliseconds
    Section[] sections = new Section[Section.COUNT]; /// Sections of the file
    // Byte total: 28+8*8=92

    /// Read the header sections, for purposes of transmitting to the PC. Returns null if invalid.
    /// Could have been a constructor but needs to return failure.
    static Header read(File file) {
        Header header = new Header();
        try (RandomAccessFile randomAccessFile = new RandomAccessFile(file, "r")) {
            randomAccessFile.readFully(header.bytes, 0, SIZE);
        } catch (IOException e) {
            Sidekick.logE("Couldn't read header for capture " + file.getAbsolutePath());
            return null;
        }
        ByteBuffer buffer = ByteBuffer.wrap(header.bytes);
        header.magic = buffer.getInt();
        header.libraryVersion = buffer.getInt();
        header.requiredAppVersion = buffer.getInt();
        header.majorErrors = buffer.getInt();
        header.minorErrors = buffer.getInt();
        header.startUnixTimeMs = buffer.getLong();
        for (int i = 0; i < Section.COUNT; i++) {
            header.sections[i] = new Section(buffer.getInt(), buffer.getInt());
        }
        return (header.magic == Signature.FILE_HEADER) ? header : null;
    }

    /// Create the capture header that will go at the beginning of the file.
    static ByteBuffer create(int majorErrors, int minorErrors, long startUnixTimeMs, Section[] sections) {
        ByteBuffer buffer = ByteBuffer.allocate(Header.SIZE);
        buffer.putInt(Signature.FILE_HEADER);
        buffer.putInt(Capture.LIBRARY_VERSION);
        buffer.putInt(Capture.FILE_MIN_APP_VERSION);
        buffer.putInt(majorErrors);
        buffer.putInt(minorErrors);
        buffer.putLong(startUnixTimeMs);
        for (Section section: sections) {
            buffer.putInt(section.offset);
            buffer.putInt(section.size);
        }
        // Pad the buffer with zeroes until it's full:
        for (int i = buffer.position(); i < Header.SIZE; i++) {
            buffer.put((byte) 0);
        }
        buffer.flip();
        return buffer;
    }
}

/// Signatures demarcate various parts of the file for additional file integrity validation.
/// Their values are chosen randomly.
class Signature {
    static final int FILE_HEADER            = 0x0BAD6005; // "BadGoos" signature
    static final int ARCHIVE_DATA           = 0xA3F7C2D8;
    static final int LOGCAT                 = 0x2E9F3D7A;
    static final int MANIFEST               = 0x7A3D9F2E;
    static final int PII                    = 0xEF82B9F3;
    static final int STRING_TOKENS          = 0xF7A9B2E1;
    static final int RECORD_DESCRIPTORS     = 0x5E4F3A9B;
    static final int DEVICE_DESCRIPTORS     = 0x9B3A4F5E;
    static final int THREAD_DESCRIPTORS     = 0x1A2B3C4D;
    static final int SERIALIZER_DESCRIPTORS = 0x4D3C2B1A;
    static final int CONFIGURATION          = 0xC0FFEE00;
}

/// Every serialized argument and return value has a parameter type to represent how it is
/// encoded. A parameter can have multiple types associated with it, as with ARRAY_PREFIX
/// and BEGIN_OBJECT/END_OBJECT which are compound formats.
class ParamType {
    static final byte VOID = 0; // For return values only
    static final byte ARRAY_PREFIX = 1; // Prefix denoting an array; the array type immediately follows
    static final byte BEGIN_OBJECT = 2; // Begin bracket for a serialized object; the serializer index immediately follows
    static final byte END_OBJECT = 3; // End bracket for serialized objects
    static final byte TO_STRING = 4; // For unrecognized types, use toString()
    static final byte STRING = 5; // Inline string
    static final byte STRING_NO_QUOTES = 6; // Inline string without quotes
    static final byte INTEGER = 7; // Signed 32-bit integer
    static final byte FLOAT = 8; // 32-bit float
    static final byte BOOLEAN = 9; // 1 byte for booleans
    static final byte ENUM = 10; // Tokenized string
}

/// IDs of reserved archive record types.
@SuppressWarnings({"unused"})
class RecordId {
    // Records without timestamps:
    static final int CHANGE_THREAD                         = 1; /// Record to denote thread changes
    static final int NOP4                                  = 2; /// No-op record with 4 bytes of data
    static final int BEGIN_BONUS                           = 3;
    static final int END_BONUS                             = 4;

    // All other records have timestamps:
    static final int TELEMETRY_SET_CAPTION                 = 8; /// AKA [RecordId#FIRST_TIMESTAMP] and [RecordId#TELEMETRY_AND_LOG_MIN]
    static final int TELEMETRY_SET_RETAINED                = 9;
    static final int TELEMETRY_REMOVE_ITEM                 = 10;
    static final int TELEMETRY_CLEAR                       = 11;
    static final int TELEMETRY_CLEAR_ALL                   = 12;
    static final int TELEMETRY_SPEAK                       = 13;
    static final int TELEMETRY_UPDATE                      = 14;
    static final int TELEMETRY_ADD_LINE                    = 15;
    static final int TELEMETRY_REMOVE_LINE                 = 16;
    static final int TELEMETRY_SET_AUTO_CLEAR              = 17;
    static final int TELEMETRY_SET_ITEM_SEPARATOR          = 18;
    static final int TELEMETRY_SET_CAPTION_VALUE_SEPARATOR = 19;
    static final int TELEMETRY_SET_DISPLAY_FORMAT          = 20;
    static final int TELEMETRY_SET_MS_TRANSMISSION_INTERVAL= 21;
    static final int LOG_SET_CAPACITY                      = 22;
    static final int LOG_SET_DISPLAY_ORDER                 = 23;
    static final int LOG_ADD_STRING                        = 24;
    static final int LOG_CLEAR                             = 25; /// AKA [RecordId#TELEMETRY_AND_LOG_MAX]

    static final int END_ARCHIVE                           = 26;
    static final int GAMEPAD1_DATA                         = 27;
    static final int GAMEPAD2_DATA                         = 28;
    static final int PAINT                                 = 29; /// [Paint] record
    static final int NOTE_STRING                           = 30; /// [Capture#note] called with a null value
    static final int LOOP                                  = 31;
    static final int SYSTEM_SAMPLES                        = 32;
    static final int MODULE0_BULK_DATA                     = 33;
    static final int MODULE1_BULK_DATA                     = 34;
    static final int MODULE2_BULK_DATA                     = 35;
    static final int MODULE3_BULK_DATA                     = 36;
    static final int PINPOINT_BULK_DATA                    = 37;
    static final int UNHANDLED_EXCEPTION                   = 38;
    static final int GLOBAL_MESSAGE                        = 39;

    // Meta data:
    static final int FIRST_TIMESTAMP = TELEMETRY_SET_CAPTION; // Records with timestamps start at this count
    static final int TELEMETRY_AND_LOG_MIN = TELEMETRY_SET_CAPTION;
    static final int TELEMETRY_AND_LOG_MAX = LOG_CLEAR;
    static final int FIRST_ALLOCATABLE = 100; // First allocatable
    static final int LAST_ALLOCATABLE = 0xffff; // Max unsigned short
}

/// Identifier of the varargs record type. These are added to [Capture.RecordDescriptor#recordId].
class VarargsType {
    static final byte NOTE = 0; /// [Sk#note]
    static final byte ALERT = 1; /// [Sk#alert]
    static final byte REQUIRE = 2; /// [Sk#require]
    static final byte TELEMETRY_ADD_DATA = 3; /// [Telemetry#addData]
    static final byte TELEMETRY_SET_VALUE = 4; /// [Telemetry.Item#setValue]
    static final byte LOG_ADD = 5; /// [Telemetry.Log#add]
    // --------------------------
    static final byte COUNT = 6;
}

/// Flags combined with [VarargsType].
@SuppressWarnings("unused")
class VarargsFlags {
    static final byte ADD_DATA_WITH_FORMAT       = (byte) 0x80; // TELEMETRY_ADD_DATA record has a format string field
    static final byte ADD_DATA_WITH_LINE_ID      = 0x40; // TELEMETRY_ADD_DATA has a line ID parent field
    static final byte ADD_DATA_COUNT_MASK        = 0xf;  // Maximum of 15 args
    static final byte SET_VALUE_WITH_FORMAT      = (byte) 0x80; // TELEMETRY_SET_VALUE record has a format string field
    static final byte SET_VALUE_PRODUCER_UPDATE  = 0x40; // TELEMETRY_SET_VALUE generated by a consumer update
    static final byte SET_VALUE_COUNT_MASK       = 0xf;  // Maximum of 15 args
}

/// Canvas flags.
class CanvasFlags {
    static final int INFERRED = 0x1; // Canvas call is inferred from a Dashboard queue
    static final int ROTATED_FIELD = 0x2; // Rendering assumes a rotated field
}

/// Minor errors indicate missing data but don't prevent captures from loading.
enum MinorError {
    CONFIGURATION,                  // 0x1: Error getting configuration file
    FTC_DASHBOARD_FIELD_OVERLAY,    // 0x2: Error getting FTC Dashboard field overlay
    LOGCAT,                         // 0x4: Error getting Logcat data
    HARDWARE_MAP,                   // 0x8: Error getting hardware map
    VOLTAGE_SENSOR,                 // 0x10: Error getting voltage sensor
    PROXY_CLASS,                    // 0x20: Error creating proxy class
    GAMEPAD,                        // 0x40: Error getting gamepad events
    BULK_READ_LYNX_MODULE,          // 0x80: Error getting bulk data from Lynx module
    CANVAS,                         // 0x100: Error getting FTC canvas data
    LIMELIGHT_VIDEO_CAPTURE,        // 0x200: Error capturing Limelight video
}

/// Major errors are so severe they prevent captures from loading.
enum MajorError {
    INTERRUPTED_CAPTURE,            // 0x1: Capture was interrupted; only Logcat is available
    DEVICE_ID_OVERFLOW,             // 0x2: Error getting device ID
    RECORD_FORMAT_TOO_LONG,         // 0x4: Error getting record format
    DEVICE_ID_TOO_LARGE,            // 0x8: Error getting record descriptor
    STRING_TOKEN_OVERFLOW,          // 0x10: More than 65535 unique string tokens
    RECORD_SIZE_TOO_LONG,           // 0x20: Record size exceeded chunk size
    FILE_READ_ERROR,                // 0x40: Fatal error reading a file
    FILE_WRITE_ERROR,               // 0x80: Fatal error writing a file
}

/// Bus types.
enum BusType {
    I2C,
    MOTOR,
    SERVO,
    ANALOG,
    DIGITAL,
    OTHER // Can be presumed to be an ethernet device
}

/// The manifest contains metadata describing the state of the capture and is saved in
/// JSON format in the capture file.
@SuppressWarnings({"unused"})
class ManifestJson {
    public String platform = "RC"; // "MRC" for Mobile Robot Controller
    public String sdkVersion = AppUtil.getSdkVersionString(); // "9.0.1"
    public int pid = android.os.Process.myPid(); // Linux process ID of the FTC app
    public String deviceModel = android.os.Build.MODEL; // Model of the Robot Controller
    public int availableProcessors = Runtime.getRuntime().availableProcessors(); // Count of CPU cores

    public boolean isBlocksOpMode = false; // True if recorded from BlocksOpMode
    public int[] suppressedIssues = new int[0]; // Array of issue codes to disable
    public int[] lynxAddresses; // Lynx port addresses, indexed by LynxModuleInfo.id
    public int pinpointDeviceVersion = 0; // GoBilda Pinpoint device version; 0 if not present
    public long initUnixTimeMs = 0; // Unix time when Init was pressed, in milliseconds
    public long initNanoTime = 0; // Nano time when Init was pressed
    public long startNanoTime = 0; // Nano time when Start was pressed
    public long endNanoTime = 0; // Nano time when Stop was pressed
    public String opModeName = ""; // "Group: Friendly name"
    public String opModeClass = ""; // "CompetitionTeleOp"
    public String appBuildTime = ""; // "2026-02-04T19:00:19.296-0800"
    public boolean[] gamepadIsPs4 = new boolean[2]; // False if gamepad is Xbox-compatible, true if PS4
    public boolean isExcessive = false; // True if the accumulated capture size was truncated
}

/// Companion to [ManifestJson] that contains privacy-sensitive data.
class PiiJson {
    public String robotName = ""; // "417-B-RC"
}

/// Structure to describe a captured thread.
class ThreadDescriptor {
    int descriptorIndex; // Index into threadDescriptorList
    String name; // Thread name
    int priority; // Java thread priority
    int tid; // Linux TID
    boolean isNew; // True if the thread was created after the opMode started
    int jiffyStart; // Jiffy count at the start of the capture; -1 if unknown
    int jiffyDuration; // Jiffies consumed during the capture; -1 if unknown (e.g., we don't have the TID)

    ThreadDescriptor(int descriptorIndex, String name, int priority, int tid, boolean isNew) {
        this.descriptorIndex = descriptorIndex;
        this.name = name;
        this.priority = priority;
        this.tid = tid;
        this.isNew = isNew;
        this.jiffyStart = (isNew) ? 0 : -1;
        this.jiffyDuration = -1;
    }
}

/// We accumulate one instance of this class for every Lynx module used.
class LynxModuleInfo {
    final Object bulkCachingLock; // Lynx module's lock for bulk caching
    LynxModule module;
    int id;
    LynxModule.BulkData lastBulkData;

    LynxModuleInfo(@NonNull Object bulkCachingLock) {
        this.bulkCachingLock = bulkCachingLock;
    }
}

/// One instance of this class is kept for every unique [HardwareDevice] class used and tracks
/// the wrapping data for serialization.
class DeviceInfo {
    int deviceId; /// Unique ID for this device class; index into [Capture#deviceClassArray]
    int maxInstances; // Maximum count of devices expected to use this class
    BusType busType; // Bus type for this device
    Class<?> originalClass; // Original device class type
    List<String> deviceNames = new LinkedList<>(); // Names of instances of this class; index by instance ID
    List<HardwareDevice> proxyInstances = new LinkedList<>(); // Proxy instances for this class; index by instance ID
    boolean isBulkReader; // True if this device is on a Lynx module that supports bulk reads
    boolean isPinpoint; /// True if the associated driver is [GoBildaPinpointDriver]
    boolean isLimelight; /// True if the associated driver is [Limelight3A]
}

/// This structure represents derivatives of device classes. An example is the [LLResult] class
/// returned by [Limelight3A#getLatestResult]; we want to capture its method calls just like we
/// do the [Limelight3A] class itself.
class DeviceChild {
    Class<?> deviceClass; // Device class, e.g., Limelight3A
    Class<?> childClass; // Derivative class, e.g., LLResult
    DeviceChild(Class<?> deviceClass, Class<?> childClass) {
        this.deviceClass = deviceClass;
        this.childClass = childClass;
    }
}

/// Principal class responsible for creating the capture.
@SuppressWarnings({"unchecked", "rawtypes", "ReassignedVariable"}) // Can add "unused"
class Capture {
    /// Descriptor for device API records and varargs records like [Sk#note]. This is not a static
    /// class as it needs to refer to the Capture object for the likes of [Capture#putFloatObject]
    /// et al.
    class RecordDescriptor {
        String methodName; // Name of the method, empty for data record descriptors
        int deviceId; /// IDs the associated device, index into index into [Capture#deviceClassArray], not used for data record descriptors
        int childId; // If non-zero, this is a child class of the associated type
        int maxInstances; // Maximum number of instances associated with this record
        int recordLength; // Length, in bytes, of the arguments for each SDK call
        Consumer[] paramPutters; // Consumers for serializing every argument
        Consumer resultPutter; // Consumer for serializing the return value
        int interceptorFlags; /// Flags indicating special handling by interceptor [Interceptor.Flags]
        int wrapId; // If the method returns a child, wrap it with this ID
        LynxModuleInfo lynxInfo; // If a Lynx bulk reader, this is the associated information

        int recordId; /// Base 16-bit identifier of this record type ([RecordId] for fixed definition records)
        byte[] formatBuffer; /// Array of [ParamType]; the formats of the arguments (and return value, if present)
        int formatIndex; /// Current parsing index into [formatBuffer]

        /// RecordDescriptor constructor for varargs calls such as [Sk#note], [Telemetry#addLine], etc.
        RecordDescriptor(SchemaKey schemaKey, int argCount) {
            methodName = ""; // Not used for varargs records; this indicates the record's type
            maxInstances = VarargsType.COUNT; // Reserve a slot for each possible type of varargs record

            recordId = nextRecordId;
            nextRecordId += maxInstances;
            if (recordId > (RecordId.LAST_ALLOCATABLE + 1)) {
                error(MajorError.DEVICE_ID_OVERFLOW);
            }

            // Remember the representation of all of the argument types:
            formatBuffer = schemaKey.format;

            // Create a 'putter' for every argument from the 'formatBuffer':
            paramPutters = new Consumer[argCount];
            for (int i = 0; i < argCount; i++) {
                paramPutters[i] = nextParamPutter(); // Updates recordLength and formatIndex
            }

            // Cache the resulting descriptor:
            varargsDescriptors.put(schemaKey, this);
            allocatedDescriptors.add(this);
        }

        /// RecordDescriptor constructor for hand-coded wrappers.
        RecordDescriptor(DeviceInfo deviceInfo, String method, SchemaKey schemaKey) {
            deviceId = deviceInfo.deviceId;
            maxInstances = deviceInfo.maxInstances;
            methodName = method;
            recordId = nextRecordId;
            nextRecordId += deviceInfo.maxInstances;
            if (recordId > (RecordId.LAST_ALLOCATABLE + 1)) {
                error(MajorError.DEVICE_ID_OVERFLOW);
            }
            formatBuffer = schemaKey.format;
            allocatedDescriptors.add(this);
            // No need to generate putters or cache the descriptor as the caller will handle that.
        }

        /// RecordDescriptor constructor for device calls wrapped via Byte Buddy.
        RecordDescriptor(Interceptor.Context context, Interceptor.MethodKey methodKey) {
            DeviceInfo deviceInfo = context.deviceInfo;
            deviceId = deviceInfo.deviceId;
            maxInstances = deviceInfo.maxInstances;
            childId = context.childId;
            methodName = methodKey.method.getName();
            recordId = nextRecordId;
            nextRecordId += deviceInfo.maxInstances;
            if (recordId > (RecordId.LAST_ALLOCATABLE + 1)) {
                error(MajorError.DEVICE_ID_OVERFLOW);
            }

            // Combine the argument and return types into a single array of parameter types:
            Class[] argumentTypes = methodKey.method.getParameterTypes();
            int argCount = argumentTypes.length;
            Class[] allParameters = Arrays.copyOf(argumentTypes, argCount + 1);
            allParameters[argCount] = methodKey.method.getReturnType();

            // Encode a representation of all of the argument types:
            SchemaKey argsKey = getSchemaKey(allParameters, true, 0);
            formatBuffer = argsKey.format;

            // Create a 'putter' for every argument plus the return value from the 'formatBuffer':
            paramPutters = new Consumer[argCount];
            for (int i = 0; i < argCount; i++) {
                paramPutters[i] = nextParamPutter(); // Updates recordLength and formatIndex
            }
            resultPutter = nextParamPutter();

            // Cache the resulting descriptor:
            methodDescriptors.put(methodKey, this);
            allocatedDescriptors.add(this);

            if ((context.deviceInfo.isBulkReader) && (BULK_READER_METHODS.contains(methodName))) {
                prepareForLynxBulkReads(context);
            } else if (context.deviceInfo.isPinpoint) {
                if (methodName.equals("setBulkReadScope")) {
                    pinpointSetBulkReadScope = true;
                }
                if (methodName.equals("update")) {
                    if (argCount == 0) {
                        /// [GoBildaPinpointDriver#update()] case:
                        interceptorFlags |= Interceptor.Flags.PINPOINT_BULK_READ;
                    } else {
                        /// [GoBildaPinpointDriver#update(GoBildaPinpointDriver.ReadData)] case:
                        interceptorFlags |= Interceptor.Flags.PINPOINT_STATUS_UPDATE;
                    }
                }
            } else if ((context.deviceInfo.isLimelight) && (methodName.equals("getLatestResult"))) {
                    interceptorFlags |= Interceptor.Flags.LIMELIGHT_STATUS_UPDATE;
            }
            for (int i = 1; i < DEVICE_CHILDREN.length; i++) { // Index of zero is reserved
                DeviceChild deviceChild = DEVICE_CHILDREN[i];
                // If the object is of the specified device class, and if the method returns the
                // specified derivative class, then wrap the result:
                if ((deviceChild.deviceClass.isAssignableFrom(context.originalObject.getClass())) &&
                    (deviceChild.childClass.isAssignableFrom(methodKey.method.getReturnType()))) {
                    interceptorFlags |= Interceptor.Flags.WRAP_CHILD;
                    wrapId = i;
                    break;
                }
            }
        }

        /// Setup for tracking Lynx module bulk reads incurred by the associated object.
        void prepareForLynxBulkReads(Interceptor.Context context) {
            // We need access to the LynxModule associated with the HardwareDevice object so that
            // we can grovel its bulk read data. We could parse the device's 'getConnectionInfo()'
            // string to attain the module address, and then compare that to each Lynx module's
            // 'getConnectionInfo' string (where we retrieve the modules via HardwareMap.getAll()).
            // However, it's faster and more straightforward to use Reflection to retrieve each
            // device's controller directly from the object.
            LynxController lynxController = null;
            Object deviceObject = context.originalObject;
            Class<?> deviceClass = deviceObject.getClass();

            if (AnalogInput.class.isAssignableFrom(deviceClass)) {
                AnalogInput analogInput = (AnalogInput) deviceObject;
                AnalogInputController analogInputController
                        = getField(analogInput, "controller", AnalogInputController.class);
                if (analogInputController != null) {
                    if (LynxAnalogInputController.class.isAssignableFrom(analogInputController.getClass())) {
                        lynxController = (LynxAnalogInputController) analogInputController;
                    }
                }
            } else if (DigitalChannelImpl.class.isAssignableFrom(deviceClass)) {
                DigitalChannelImpl digitalChannel = (DigitalChannelImpl) deviceObject;
                DigitalChannelController digitalChannelController
                        = getField(digitalChannel, "controller", DigitalChannelController.class);
                if (digitalChannelController != null) {
                    if (LynxDigitalChannelController.class.isAssignableFrom(digitalChannelController.getClass())) {
                        lynxController = (LynxDigitalChannelController) digitalChannelController;
                    }
                }
            } else if (DcMotorImpl.class.isAssignableFrom(deviceClass)) {
                DcMotorImpl dcMotor = (DcMotorImpl) deviceObject;
                DcMotorController dcMotorController = dcMotor.getController();
                if (dcMotorController != null) {
                    if (LynxDcMotorController.class.isAssignableFrom(dcMotorController.getClass())) {
                        lynxController = (LynxDcMotorController) dcMotorController;
                    }
                }
            }

            LynxModule lynxModule = getField(lynxController, "module", LynxModule.class);
            if (lynxModule == null) {
                error(MinorError.BULK_READ_LYNX_MODULE);
                return; // ====>
            }

            lynxInfo = lynxModuleMap.get(lynxModule);
            if (lynxInfo != null) { // The error is already marked if 'lynxModuleMap' is incomplete
                interceptorFlags |= Interceptor.Flags.LYNX_BULK_READ;
            }
        }

        /// This is another argument consumer similar to [Capture#putIntegerObject], et al, except
        /// that it comes bundled with additional state to handle the complexity of serializing an
        /// array.
        class ArrayPutter {
            static final int MAX_ARRAY_SIZE = 250; // Count fits in an unsigned byte

            Consumer paramPutter; // 'putter' for the type of the array's elements
            int argSize; // The byte size of an element of the array

            // Take a 'putter' for the array's elements.
            ArrayPutter(Consumer paramPutter, int argSize) {
                this.paramPutter = paramPutter;
                this.argSize = argSize;
            }

            // Serialize an array by writing its size and then putting each element.
            void putArray(Object arg) {
                Object[] array = (Object[]) arg;
                int count = Math.min(array.length, MAX_ARRAY_SIZE);
                chunk.put((byte) count); // This doesn't need to check chunkRemaining; already done
                for (int i = 0; i < count; i++) {
                    // To reduce fragmentation and also avoid the problem of count * argSize >
                    // CHUNK_SIZE, we check the space on every loop:
                    if ((chunkRemaining -= argSize) < 0) {
                        nextChunk();
                    }
                    paramPutter.accept(array[i]);
                }
            }
        }

        /// This is another argument consumer similar to [Capture#putIntegerObject], et al, except
        /// that it comes bundled with additional state to handle the complexity of serializing a
        /// class object.
        class ObjectPutter {
            final Sidekick.SerializerInfo serializerInfo;
            final Consumer[] putters;

            /// Create the putters for the fields of an object.
            ObjectPutter(Sidekick.SerializerInfo serializerInfo) {
                int argCount = serializerInfo.prototypeOutput.length;
                this.serializerInfo = serializerInfo;
                this.putters = new Consumer[argCount];
                for (int i = 0; i < argCount; i++) {
                    putters[i] = nextParamPutter();
                }
            }

            /// Serialize an object (as opposed to a primitive type).
            void putObject(Object arg) {
                byte isNonNull;
                Object[] values;

                // Records of the same type always have to be the same length, regardless of
                // where an object is null or not. So if an object is null, we fill it out
                // with fake values.
                if (arg == null) {
                    isNonNull = 0;
                    values = serializerInfo.prototypeOutput; // Fake values
                } else {
                    isNonNull = 1;
                    values = serializerInfo.serializer.serialize(arg);
                }
                chunk.put(isNonNull); // One byte to indicate whether it's null or not
                for (int i = 0; i < putters.length; i++) {
                    putters[i].accept(values[i]);
                }
            }
        }

        /// Adds to [RecordDescriptor#recordLength] on every invocation.
        Consumer nextParamPutter() {
            switch (formatBuffer[formatIndex++]) {
                case ParamType.INTEGER: recordLength += 4; return Capture.this::putIntegerObject;
                case ParamType.FLOAT: recordLength += 4; return Capture.this::putFloatObject;
                case ParamType.BOOLEAN: recordLength += 1; return Capture.this::putBooleanObject;
                case ParamType.ENUM: recordLength += 2; return Capture.this::putEnumObject;
                case ParamType.TO_STRING: return Capture.this::putToStringObject;
                case ParamType.STRING: return Capture.this::putStringObject; // String does its own length check
                case ParamType.STRING_NO_QUOTES: return Capture.this::putStringObject;
                case ParamType.VOID: return Capture.this::putVoidObject;
                case ParamType.ARRAY_PREFIX:
                    int arrayStartLength = ++recordLength; // Reserve a byte for the array size
                    Consumer paramPutter = nextParamPutter(); // Increments recordLength
                    int argLength = recordLength - arrayStartLength;
                    recordLength = arrayStartLength;
                    return new RecordDescriptor.ArrayPutter(paramPutter, argLength)::putArray;
                case ParamType.BEGIN_OBJECT:
                    int serializerIndex = formatBuffer[formatIndex++];
                    Sidekick.SerializerInfo serializerInfo = sidekick.serializersList.get(serializerIndex);
                    assert (serializerInfo != null);
                    RecordDescriptor.ObjectPutter objectPutter = new RecordDescriptor.ObjectPutter(serializerInfo);
                    recordLength++; // Reserve the null-object indicator byte
                    assert(formatBuffer[formatIndex] == ParamType.END_OBJECT);
                    formatIndex++;
                    return objectPutter::putObject;
            }
            return null;
        }
    }

    /// Version numbers are 32-bit values encoded as major.minor.build:
    @SuppressWarnings("SameParameterValue")
    static int CreateVersion(int major, int minor, int build) {
        return (major << 24) | ( minor << 12) | build;
    }

    // This library's current version number is:
    final static int LIBRARY_VERSION = CreateVersion(10, 0, 1);

    // To load a file created by this library, the app has to have at least this version number:
    final static int FILE_MIN_APP_VERSION = CreateVersion(0, 0, 1);

    // To communicate via sockets with this library, the app needs this version number or better:
    final static int SOCKET_MIN_APP_VERSION = CreateVersion(0, 0, 1);

    // Constants:
    final static boolean DEBUG = true; // Enable debug code
    final static int TICK_SHIFT = 8; // Downshift nanoTime() by this many bits to encode time ticks
    final static int TICK_MASK = 0xffffff; // Mask to extract time ticks from nanoTime() after shift
    final static byte NULL_TERMINATOR = 0; // We null-terminate all strings in the archive
    final static byte NULL_STRING_MARKER = (byte) 0x80; // Byte encoding to represent null strings
    final static int DEFAULT_MAX_STRING_BYTES = 512; // Strings are truncated to this length, in bytes
    final static int STRING_TOKEN_LAST_ALLOCATABLE_ID = Short.MAX_VALUE; // Max string token ID
    final static int BULK_DATA_BYTE_COUNT = 34; // The system's bulk data payload is 34 bytes
    final static int MAX_LYNX_COUNT = 4; // Maximum number of Lynx modules for bulk reads
    final static HashSet<String> BULK_READER_METHODS = // Methods that can do Lynx module bulk reads
            new HashSet<>(Arrays.asList("getCurrentPosition", "isBusy", "getVelocity", "isOverCurrent", "getVoltage", "getState"));
    final static DeviceChild[] DEVICE_CHILDREN = { // Hardware device children that will be wrapped
            new DeviceChild(Void.class, Void.class), // 0 entry reserved for non-child devices
            new DeviceChild(Limelight3A.class, LLResult.class),
            new DeviceChild(Limelight3A.class, LLStatus.class),
            new DeviceChild(Servo.class, ServoController.class),
            new DeviceChild(CRServo.class, ServoController.class),
    };
    final Object sidekickLock; // Master synchronization object, same lock object as Sidekick
    final Charset UTF8 = StandardCharsets.UTF_8; // Our standard charset for strings
    final CharsetEncoder utf8Encoder = UTF8.newEncoder();

    // Statics:
    static Capture instance; // Active capture; null before first opMode or when Sidekick is disabled
    static AtomicInteger minBatteryMilliVolts = new AtomicInteger(Integer.MAX_VALUE); // Min voltage read since last broadcast, mV, MAX_VALUE if none
    static volatile int lastBatteryMilliVolts; // The most recent battery voltage read, mV

    // Instanced data:
    boolean isEnded; // True if done capturing and nothing more should be recorded
    boolean isExcessive; // True if the capture size is excessive and the capture should be ended
    ManifestJson manifest = new ManifestJson(); // State compendium saved with capture
    PiiJson pii = new PiiJson(); // Privacy-sensitive data saved with capture
    int minorErrors; /// Bitmask of [MinorError] capture issues seen that are non-fatal
    int majorErrors; /// Bitmask of [MajorError] capture issues seen that are fatal for the capture
    int nextRecordId = RecordId.FIRST_ALLOCATABLE; // ID of next record descriptor to be created
    int nextStringId = 1; // ID of next string token to be created; zero is reserved
    Sidekick sidekick; // Reference to the corresponding Sidekick object
    OpMode opMode; // Reference to the currently active opMode
    SkHardwareMap hardwareMap; // Sidekick's enhanced hardware map
    SkTelemetry telemetry; // Sidekick's enhanced telemetry object
    ByteBuffer chunk; // Current chunk given by Persistence for writing data to
    int chunkRemaining; // Amount of bytes remaining in the current chunk
    Date initDate; // Date and time at which the OpMode was initialized
    long initNanoTime; // Capture init time, in nanoseconds
    long initUnixTime; // Unix time when Init was pressed, in milliseconds
    long startNanoTime; // Capture start time, in nanoseconds; same as init time if not started
    long startUnixTime; // Unix time when Start was pressed, in milliseconds
    volatile boolean isStarted; // True if the opMode has started (Start was pressed)
    long endNanoTime; // Capture end time, in nanoseconds
    String dateAndTime; // Date and time at which the OpMode was initialized, used for file names
    Section[] sections; // Data on all capture sections
    String logcatFilename; // Name of the logcat file
    Process logcatProcess; // Process that records Logcat data during the capture
    int opModeTid; // Linux TID of the OpMode thread; zero if not yet known
    long currentThreadId; /// Java ID of most recent thread to write to the archive (not the
        /// Linux TID!). Zero is a reserved value that causes [#threadSwitchAndBookkeeping] to be
        /// called before the next record.
    boolean[] gamepadIsPs4 = new boolean[2]; // False if gamepad is Xbox-compatible, true if PS4
    ArrayList<DeviceInfo> deviceClassArray = new ArrayList<>(); /// List of all device classes, index by [DeviceInfo#deviceId]
    HashMap<Interceptor.MethodKey, RecordDescriptor> methodDescriptors = new HashMap<>(); // Descriptor map for methods
    Map<SchemaKey, RecordDescriptor> varargsDescriptors = new HashMap<>(); // Descriptor map for varargs
    HashMap<String, RecordDescriptor> hardwareMapDescriptors = new HashMap<>(); /// For [#recordHardwareMapGet]
    LinkedList<RecordDescriptor> allocatedDescriptors = new LinkedList<>(); // List of all descriptors
    HashMap<Thread, ThreadDescriptor> threadDescriptorMap = new HashMap<>(); // Map thread to descriptor
    LinkedList<ThreadDescriptor> threadDescriptorList = new LinkedList<>(); // List of all descriptors, zero is reserved
    HashMap<LynxModule, LynxModuleInfo> lynxModuleMap = new HashMap<>(); // Not all Lynx modules may be present
    int processJiffyStart; // The process's jiffy count at the start of the capture
    int processJiffyDuration; // The process's total jiffy count, determined at the end of the capture
    long wallClockJiffyStart; // The start timer for wallClockJiffyDuration, in nano seconds
    int wallClockJiffyDuration; // The wall clock's total jiffy count, in jiffies, for a single core
    Field lynxModuleLastBulkDataField; /// LynxModule.lastBulkData (type [LynxModule.BulkData])
    Field bulkDataRespField; /// LynxModule.BulkData.resp (type [LynxGetBulkInputDataResponse])
    HashMap<String, Integer> stringTokensMap = new HashMap<>(); // Static string map
    LinkedList<String> stringTokensList = new LinkedList<>(); // Static string list
    int uniquenessCounter = 0; /// Used by [Capture#generateUniqueness()]
    final Map<Class<?>, SchemaKey> classParamTypes = new HashMap<>(); // Stash of param types for varargs
    byte[][] lastLynxBulkPayload = new byte[MAX_LYNX_COUNT][BULK_DATA_BYTE_COUNT]; // Last recorded bulk payload
    boolean pinpointSetBulkReadScope = false; /// True if [GoBildaPinpointDriver#setBulkReadScope] has been called
    CountDownLatch fileDoneLatch = new CountDownLatch(1); // Signals when the file is done being written
    boolean usingCamera = false; // True if using webcam or built-in camera (not a Limelight)
    String limelightAddress; // IP address of a Limelight camera found during Init, null if none
    LimelightVideo limelightCapture; // Video capture for the Limelight camera, null if not started

    /// Generate a unique ID for a record. Guaranteed to be non-zero even if truncated to 16 bits.
    int generateUniqueness() {
        if ((++uniquenessCounter & 0xffff) == 0) {
            ++uniquenessCounter;
        }
        return uniquenessCounter;
    }

    /// Record errors.
    void error(MinorError error) { error(error, null); }
    void error(MinorError error, @Nullable String message, Object... args) {
        minorErrors |= (1 << error.ordinal());
        Sidekick.logW(message != null ? String.format(message, args) : String.format("Minor error: %s", error));
    }
    void error(MajorError error) { error(error, null); }
    void error(MajorError error, @Nullable String message, Object... args) {
        majorErrors |= (1 << error.ordinal());
        Sidekick.logE(message != null ? String.format(message, args) : String.format("Major error: %s", error));
    }

    /// The Capture's only constructor.
    Capture(Sidekick sidekick, @NonNull OpMode opMode) {
        this.sidekick = sidekick;
        this.opMode = opMode;

        this.sidekickLock = Sidekick.sidekickLock;
        this.currentThreadId = Thread.currentThread().getId();
        this.initDate = new Date();
        this.initNanoTime = nanoTime();
        this.initUnixTime = initDate.getTime();
        this.startNanoTime = initNanoTime; /// This will get overwritten if/when [#markStartTime()] gets called
        this.startUnixTime = initUnixTime;
        SimpleDateFormat dateFormatter = new SimpleDateFormat("yyyy-MM-dd '['HH.mmaa']'", Locale.US);
        this.dateAndTime = dateFormatter.format(initDate).toLowerCase(); // Lower 'AM' to 'am'

        // Initialize the array of file sections to zero:
        this.sections = new Section[Section.COUNT];
        for (int i = 0; i < Section.COUNT; i++) {
            this.sections[i] = new Section(0, 0);
        }

        // Initialize the 0th entry of the thread descriptor list to be reserved:
        threadDescriptorList.add(new ThreadDescriptor(0, "reserved", 0, 0, false));
        snapshotThreadsAndJiffies(true);
    }

    /// Register a new thread.
    ThreadDescriptor registerThread(Thread thread, String name, int tid, boolean isNew) {
        if (name.equals("OpModeThread")) {
            opModeTid = android.os.Process.myTid();
        }

        ThreadDescriptor descriptor = new ThreadDescriptor(threadDescriptorList.size(), name,
                thread.getPriority(), tid, isNew);
        threadDescriptorMap.put(thread, descriptor);
        threadDescriptorList.add(descriptor);
        return descriptor;
    }

    /// Get the current process jiffy count and add a snapshot of active Java threads. `start`
    /// is true if the snapshot is happening at the beginning of the capture; false if at the end.
    void snapshotThreadsAndJiffies(boolean start) {
        double startMillis = System.nanoTime() * 1e-6;
        if (start) {
            processJiffyStart = ThreadSampler.getProcessJiffies();
            wallClockJiffyStart = nanoTime();
        } else {
            // Every jiffy is 10ms of wall clock time:
            processJiffyDuration = ThreadSampler.getProcessJiffies() - processJiffyStart;
            wallClockJiffyDuration = (int)((nanoTime() - wallClockJiffyStart) * 1e-7);
        }

        ThreadGroup root = Thread.currentThread().getThreadGroup();
        assert(root != null);
        while (root.getParent() != null) {
            root = root.getParent();
        }
        Thread[] threads = new Thread[root.activeCount()];
        root.enumerate(threads, true); // include subgroups
        for (Thread thread: threads) {
            ThreadDescriptor descriptor = threadDescriptorMap.get(thread);
            if (descriptor == null) {
                int tid = ThreadPool.getTID(thread);
                String name = thread.getName();

                // Override the name and TID if this thread is in the Sidekick thread registry:
                Sidekick.ThreadRegister threadRegister = Sidekick.instance.threadRegistry.get(thread);
                if (threadRegister != null) {
                    name = threadRegister.name;
                    tid = threadRegister.tid;
                }

                descriptor = registerThread(thread, name, tid, !start);
            }

            if (start) {
                // We're at the start of the run, so record the jiffies that the thread has already
                // consumed:
                descriptor.jiffyStart = ThreadSampler.getThreadJiffies(descriptor.tid);
            } else {
                // We're at the end of the run and can compute the thread's duration. However,
                // don't do it if the duration has already been computed (i.e., the thread ended):
                if (descriptor.jiffyDuration == -1) {
                    // Calculate the duration when both the start and end jiffies are known:
                    int jiffies = ThreadSampler.getThreadJiffies(descriptor.tid);
                    if ((jiffies != -1) && (descriptor.jiffyStart != -1)) {
                        descriptor.jiffyDuration = jiffies - descriptor.jiffyStart;
                    }
                }
            }
        }
        double duration = System.nanoTime() * 1e-6 - startMillis;
        Sidekick.logI("Snapshotting threads took %.2f ms", duration);
    }

    /// Called by Sidekick to register a start/end transition of the specified thread.
    void registerThreadTransition(Thread thread, Sidekick.ThreadRegister register, boolean start) {
        Sidekick.logI("Registering transition for thread '%s': %s", thread.getName(), start); // @@@

        assert(Thread.holdsLock(Sidekick.sidekickLock));
        ThreadDescriptor descriptor = threadDescriptorMap.get(thread);
        if (start) {
            // The thread may already be registered so check:
            if (descriptor == null) {
                registerThread(thread, register.name, register.tid, true);
            }
        } else {
            if (descriptor != null) {
                // We have to query the thread's jiffies before the thread terminates so do it now:
                if (descriptor.jiffyStart != -1) {
                    int jiffies = ThreadSampler.getThreadJiffies(register.tid);
                    if (jiffies != -1) {
                        descriptor.jiffyDuration = jiffies - descriptor.jiffyStart;
                    }
                }
            }
        }
    }

    /// Reflection helper to get a field value; works on fields of superclasses.
    static <T> T getField(Object object, String fieldName, Class<T> type) {
        if (object == null)
            return null;
        Class<?> currentClass = object.getClass();
        while (currentClass != null) {
            try {
                Field f = currentClass.getDeclaredField(fieldName);
                f.setAccessible(true);
                Object value = f.get(object);
                return type.cast(value);
            } catch (NoSuchFieldException ignored) {
                currentClass = currentClass.getSuperclass();
            } catch (IllegalAccessException e) {
                return null;
            }
        }
        return null; // not found anywhere
    }

    /// Returns the [ParamType] serialization format for the given class.
    static byte getParamType(Class<?> klass) {
        String name = klass.getName();
        switch (name) {
            // Method parameters (generally) have unboxed types, but parameters returned from
            // serializers are always boxed, and a new device API might have boxed arguments.
            case "void":
            case "java.lang.Void":
                return ParamType.VOID;
            case "double":
            case "float":
            case "java.lang.Double":
            case "java.lang.Float":
                return ParamType.FLOAT;
            case "int":
            case "long":
            case "short":
            case "byte":
            case "char":
            case "java.lang.Integer":
            case "java.lang.Long":
            case "java.lang.Short":
            case "java.lang.Byte":
            case "java.lang.Char":
                return ParamType.INTEGER;
            case "boolean":
            case "java.lang.Boolean":
                return ParamType.BOOLEAN;
            case "java.lang.String":
                return ParamType.STRING;
            default:
                if (klass.isEnum()) {
                    return ParamType.ENUM;
                } else if (klass.isArray()) {
                    return ParamType.ARRAY_PREFIX;
                } else if (Sidekick.instance.serializersMap.get(klass) != null) {
                    return ParamType.BEGIN_OBJECT;
                } else {
                    // Our catch-all is to convert the parameter to a String using 'toString()'.
                    return ParamType.TO_STRING;
                }
        }
    }

     /// This is a record wrapper for the varargs format byte array so that it can be used as a key
     /// in a Java Map.
     ///
     /// The varargs format is an array of [ParamType] bytes; generally, it's one per argument. If a
     /// function takes 2 float arguments with no return value, the buffer would be ["FLOAT, FLOAT"].
     /// If it has a return value, that type gets appended to the end. The encoding for serialized
     /// objects is more complicated so as to represent a hierarchy with object types. For example,
     /// if a function takes a Pose2D and an int as arguments, the buffer would be:
     /// ["BEGIN_OBJECT, {Pose2D-identifier}, FLOAT, FLOAT, FLOAT, END_OBJECT, INTEGER"]
    static class SchemaKey {
        public byte[] format; /// Array of [ParamType]
        SchemaKey(byte[] format) {
            this.format = format;
        }

        @Override public boolean equals(Object o) {
            if (this == o)
                return true;
            if (!(o instanceof SchemaKey)) // Yes, this check is necessary
                return false;
            SchemaKey other = (SchemaKey) o;
            return Arrays.equals(this.format, other.format);
        }

        @Override public int hashCode() {
            return Arrays.hashCode(this.format);
        }
    }

    /// Efficiently return the serialization formats for each of the given arguments. When
    /// encoding varargs, the provided args array will be the actual varargs array of objects,
    /// and `argsAreClasses` will be false. When encoding methods, the args array will be the
    /// [Method#getParameterTypes] array of Class<?> for each argument of the method, and
    /// `argsAreClasses` will be true.
    @NonNull
    SchemaKey getSchemaKey(Object[] args, boolean argsAreClasses, int recursionLevel) {
        int argCount = args.length;
        byte[] resultBuffer = new byte[argCount]; // Size array assuming no objects, for now
        int resultIndex = 0;
        for (Object arg: args) {
            SchemaKey schemaKey;
            if (arg == null) {
                schemaKey = new SchemaKey(new byte[]{ParamType.VOID});
            } else {
                Class<?> klass = (argsAreClasses) ? (Class<?>) arg : arg.getClass();
                schemaKey = classParamTypes.get(klass); // Look for this argument's class in the map
                if (schemaKey == null) {
                    // Couldn't find precomputed param types for this class, so compute them
                    // now and cache the result for next time:
                    byte paramType = getParamType(klass);
                    if (paramType == ParamType.ARRAY_PREFIX) {
                        // Get the schema key for the type of the array's elements:
                        SchemaKey arrayKey = getSchemaKey(new Class[]{klass.getComponentType()}, true, recursionLevel);

                        // Prepend ARRAY_PREFIX to the schema; that's the key for this argument.
                        byte[] arrayKeyBuffer = new byte[arrayKey.format.length + 1];
                        arrayKeyBuffer[0] = ParamType.ARRAY_PREFIX;
                        System.arraycopy(arrayKey.format, 0, arrayKeyBuffer, 1, arrayKey.format.length);
                        schemaKey = new SchemaKey(arrayKeyBuffer);

                    } else if (paramType == ParamType.BEGIN_OBJECT) {
                        // Get the schema key for the object:
                        Sidekick.SerializerInfo<?> serializerInfo = sidekick.serializersMap.get(klass);
                        assert (serializerInfo != null);
                        SchemaKey objectKey = getSchemaKey(serializerInfo.prototypeOutput, false, recursionLevel + 1);

                        // Bracket the schema with BEGIN/END_OBJECT; that's the key for this argument:
                        byte[] objectFormatBuffer = new byte[objectKey.format.length + 3];
                        objectFormatBuffer[0] = ParamType.BEGIN_OBJECT;
                        objectFormatBuffer[1] = (byte) serializerInfo.identifier; // @@@ Cap serializer count
                        System.arraycopy(objectKey.format, 0, objectFormatBuffer, 2, objectKey.format.length);
                        objectFormatBuffer[objectFormatBuffer.length - 1] = ParamType.END_OBJECT;
                        schemaKey = new SchemaKey(objectFormatBuffer);

                    } else {
                        // This argument has a simple type:
                        schemaKey = new SchemaKey(new byte[]{paramType});
                    }

                    // Cache the result in the map for next time:
                    classParamTypes.put(klass, schemaKey);
                }
                // If it's an object, make sure the added argument count doesn't exceed the maximum:
                if (schemaKey.format.length > 4) {
                    // 3 for begin/id/end bytes, and 1 argument in the object is already accounted for:
                    argCount += schemaKey.format.length - 4; // @@@ Gotta actually use argCount and check max length - or delete it
                }
            }
            // Special case the typical, simple case:
            if (schemaKey.format.length == 1) {
                resultBuffer[resultIndex] = schemaKey.format[0];
                resultIndex++;
            } else {
                // Don't allow objects beyond the first level; use toString() at lower levels:
                if ((schemaKey.format[0] == ParamType.BEGIN_OBJECT) && (recursionLevel > 0)) {
                    schemaKey = new SchemaKey(new byte[]{ParamType.STRING});
                }
                // Resize the results array, remembering that we expected this to take one byte:
                byte[] newResult = new byte[resultBuffer.length + schemaKey.format.length - 1];
                System.arraycopy(resultBuffer, 0, newResult, 0, resultIndex);
                System.arraycopy(schemaKey.format, 0, newResult, resultIndex, schemaKey.format.length);
                resultBuffer = newResult;
                resultIndex += schemaKey.format.length;
            }
        }
        return new SchemaKey(resultBuffer);
    }

    /// Remember the time of the 'Start' button. We get notified by the event loop listener.
    void markStartTime() {
        isStarted = true;
        startNanoTime = nanoTime();
        startUnixTime = System.currentTimeMillis();
        Sidekick.logI("\u25B6 was pressed"); /// This message syncs [#startNanoTime] with Unix time

        // Video recording files have the same root name as the capture; only the extension differs:
        String videoRootName = Sidekick.SUBDIRECTORY + "/" + dateAndTime;

        // Start Limelight video capture if a Limelight was registered during Init:
        if (limelightAddress != null) {
            limelightCapture = new LimelightVideo(videoRootName + ".lvc", limelightAddress);
        }

        // Ask a connected PC to potentially start a screen capture via ADB:
        int timeLimit = opMode.getClass().isAnnotationPresent(TeleOp.class) ? 120 : 30; // Seconds
        Socket.broadcastStartScreenRecord(videoRootName + ".mp4", usingCamera, timeLimit);
    }

    /// Return the total amount of bytes queued for writing to the file.
    int getOffset() {
        return chunk.capacity() - chunk.remaining() + sidekick.fileWorker.getBytesQueued();
    }

    /// Do some book keeping and record that a thread switch occurred. Return true if everything
    /// is good; false if we are no longer recording and the caller should abort.
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    boolean threadSwitchAndBookkeeping() {
        assert(Thread.holdsLock(sidekickLock));
        if (isEnded) {
            return false; // ====>
        }
        // If the capture size is excessive, this is a safe place to end it:
        if (isExcessive) {
            endCapture();
            return false; // ====>
        }

        /// Update [#currentThreadId] and find the descriptor for the current thread:
        Thread currentThread = Thread.currentThread();
        currentThreadId = currentThread.getId();

        ThreadDescriptor descriptor = threadDescriptorMap.get(currentThread);
        if (descriptor == null) {
            // This thread didn't exist at the start of the capture so create a descriptor:
            descriptor = registerThread(currentThread, currentThread.getName(),
                    android.os.Process.myTid(), true);
        }

        /// The descriptor's TID can be zero if the thread existed at startup but wasn't from
        /// [ThreadPool]. If that's the case, we can query it now:
        if (descriptor.tid == 0) {
            descriptor.tid = android.os.Process.myTid();
        }

        // Don't save the time as this record is effectively just a prefix of the subsequent
        // record - which will have the time.
        if ((chunkRemaining -= 4) < 0) {
            nextChunk();
        }
        chunk.putShort((short) RecordId.CHANGE_THREAD);
        chunk.putShort((short) descriptor.descriptorIndex);
        assertChunk();
        return true;
    }

    /// Queue the current chunk for write and replace it with a new chunk that can be written
    /// to immediately. This routine does not block on the I/O write.
    ///
    /// Note: The caller must ensure that their request is never bigger than the hardcoded
    ///       chunk size.
    void nextChunk() {
        if (DEBUG) {
            assert(!isEnded); // Caller should have checked for 'stopRecording'!
            assert(Thread.holdsLock(sidekickLock)); // Caller should have done "synchronized (lock)"
        }

        // Reconstitute the size that the caller needs to write, knowing that bufferRemaining
        // was pre-decremented:
        int requestedSize = (chunk != null) ? chunk.remaining() - chunkRemaining : -chunkRemaining;

        // Queue the buffer for write and at the same time get a new buffer. Calculate the
        // new remainder:
        chunk = sidekick.fileWorker.writeAndFlipChunk(chunk);
        chunkRemaining = chunk.remaining() - requestedSize;
        if (chunkRemaining < 0) {
            // Well, the caller violated the contract about requests bigger than the chunk
            // size. Rather than terminating the program now, we somewhat more gracefully
            // supply a buffer that's big enough for the request:
            chunk = ByteBuffer.allocateDirect(requestedSize);
            chunkRemaining = 0;
            error(MajorError.RECORD_SIZE_TOO_LONG);
        }

        /// Check to see if the file size is excessive and the capture should be ended. We can't
        /// call [#endCapture()] here because the caller is in the midst of writing a record;
        /// instead, we provoke [#threadSwitchAndBookkeeping()] to be called at the start of the next record,
        /// and that will end it.
        if (sidekick.fileWorker.isExcessive()) {
            currentThreadId = 0; /// Provoke [#threadSwitchAndBookkeeping] when the next record is to be added
            isExcessive = true;
        }
    }

    /// NOTE: The caller must check the return code and early out on failure!
    boolean beginRecord(int recordId, int payloadByteCount) {
        return beginRecord(recordId, payloadByteCount, nanoTime());
    }
    boolean beginRecord(int recordId, int payloadByteCount, long nanoTime) {
        assert(Thread.holdsLock(sidekickLock));
        if (Thread.currentThread().getId() != currentThreadId) {
            if (!threadSwitchAndBookkeeping())
                return false; // ====> Don't record any more
        }
        if ((chunkRemaining -= (5 + payloadByteCount)) < 0) {
            nextChunk();
        }
        chunk.putShort((short) recordId);
        long tickTime = nanoTime >> TICK_SHIFT;
        chunk.put((byte) (tickTime >> 16));
        chunk.putShort((short) (tickTime));
        return true;
    }

    /// This is called to save a null String parameter. To differentiate from an empty string,
    /// this represents the null as an invalid string that is properly null-terminated. String
    /// readers have to check for this special case.
    void putNullString() {
        if ((chunkRemaining -= 2) < 0) {
            nextChunk();
        }
        chunk.put(NULL_STRING_MARKER);
        chunk.put(NULL_TERMINATOR);
    }

    /// Add a string to the chunk, with a null terminator. NOTE: The caller doesn't need to check
    /// for space in the buffer; it can omit the added string from its `bytesRemaining` adjustment.
    void putString(@NonNull String string) { putString(string, DEFAULT_MAX_STRING_BYTES); }
    void putString(@NonNull String string, int maxByteSize) {
        int totalBytesWritten = 0;

        // Wrap the string. This object remembers its position, so if we stop halfway through,
        // it knows where to resume:
        CharBuffer inBuffer = CharBuffer.wrap(string);
        utf8Encoder.reset();

        // Loop until the string is fully encoded OR we hit the max byte limit:
        while (true) {
            // Calculate how many bytes we can write right now. It is the smaller of:
            //   A. The space left in the current physical chunk (chunkRemaining)
            //   B. The space left in our logical byte limit
            int spaceInChunk = chunk.remaining();
            int spaceAllowed = maxByteSize - totalBytesWritten;
            int writeLimit = Math.min(spaceInChunk, spaceAllowed);

            // Constrain the chunk so the encoder doesn't write too much:
            int startPosition = chunk.position();
            int originalLimit = chunk.limit();
            chunk.limit(startPosition + writeLimit);

            // Write as much as fits into the 'writeLimit'. It automatically advances inBuffer's
            // position:
            utf8Encoder.encode(inBuffer, chunk, true);

            // Restore the chunk's true limit:
            chunk.limit(originalLimit);

            // Update trackers:
            int bytesWritten = chunk.position() - startPosition;
            totalBytesWritten += bytesWritten;
            chunkRemaining -= bytesWritten;

            /// We fudge the [totalBytesWritten] comparison value because we might otherwise get
            /// within 1 byte of the end but be unable to add a multi-byte character - which would
            /// cause us to loop endlessly:
            if ((totalBytesWritten >= maxByteSize - 3) || (!inBuffer.hasRemaining())) {
                break;
            }

            // The loop continues. We pass the SAME inBuffer to encode() again, and it resumes
            // automatically.
            nextChunk();
        }

        // 3. Write the terminator:
        if ((chunkRemaining -= 1) < 0) {
            nextChunk();
        }
        chunk.put(NULL_TERMINATOR);
    }

    /// Add a 24-bit value to the chunk, in big-endian format.
    void putInt24(int value) {
        chunk.put((byte) (value >> 16));
        chunk.putShort((short) (value));
    }

    /// Add a byte array to the capture.
    void putBytes(byte[] data) { putBytes(data, data.length); }
    void putBytes(byte[] data, int size) {
        int offset = 0;
        while (true) {
            int length = Math.min(chunkRemaining, size - offset);
            if (length == 0) {
                return; // ====>
            }
            chunk.put(data, offset, length);
            offset += length;
            if ((chunkRemaining -= length) == 0) {
                nextChunk();
            }
        }
    }

    /// Debug routine that can be called after writing a record to ensure that the manual
    /// calculation of the record size is consistent with what was actually written to the buffer.
    void assertChunk() {
        assert(Thread.holdsLock(sidekickLock)); // Caller should have done "synchronized (lock)"
        if (chunkRemaining != chunk.remaining()) {
            throw new RuntimeException(String.format("chunkRemaining %d != chunk.remaining() %d",
                    chunkRemaining, chunk.remaining()));
        }
    }

    /// Record a string in the token table and return a 16-bit identifier token. This must be
    /// used only for strings that are reasonably expected to be static, such as Enum and
    /// [Sk#note] identifiers, otherwise the token table will get blown out.
    short tokenizeString(String staticString) {
        // Lookup the string:
        Integer tokenId = stringTokensMap.get(staticString);
        if (tokenId != null)
            return (short) (int) tokenId;

        // Allocate the string:
        tokenId = nextStringId;
        if (tokenId > STRING_TOKEN_LAST_ALLOCATABLE_ID) {
            error(MajorError.STRING_TOKEN_OVERFLOW);
            return STRING_TOKEN_LAST_ALLOCATABLE_ID;
        }
        nextStringId++;
        stringTokensMap.put(staticString, tokenId);
        stringTokensList.add(staticString);
        return (short) (int) tokenId;
    }

    /// Remove all of the prefixes of the type name.
    String rootName(String typeName) {
        int index = typeName.lastIndexOf('.');
        return (index == -1) ? typeName : typeName.substring(index + 1);
    }

    /// When serializing descriptors of hardware device objects, we want to save the name of the
    /// class as its canonical SDK name. For example, [DcMotorEx] objects are actually of type
    /// "[DcMotorImplEx]" which is internal ugliness, so substitute "DcMotorEx" instead..
    String getSdkName(Class<?> klass) {
        // First, search through 'allDeviceMappings' for a matching type:
        for (HardwareMap.DeviceMapping<?> deviceMapping: hardwareMap.allDeviceMappings) {
            if (deviceMapping.getDeviceTypeClass().isAssignableFrom(klass)) {
                return deviceMapping.getDeviceTypeClass().getName();
            }
        }

        // The HardwareMap list isn't exhaustive so check a supplemental list:
        final Class<?>[] canonicalClasses = {IMU.class, VoltageSensor.class, CRServo.class, Servo.class,
                DcMotor.class, NormalizedColorSensor.class, DistanceSensor.class, DigitalChannel.class,
                WebcamName.class, AnalogInput.class, SparkFunOTOS.class, GoBildaPinpointDriver.class,
                LED.class, Limelight3A.class};
        for (Class<?> canonicalClass: canonicalClasses) {
            if (canonicalClass.isAssignableFrom(klass))
                return canonicalClass.getName();
        }

        // It's no big deal that we didn't find a match; the name may just appear uglier in the UI:
        return klass.getName();
    }

    /// Remember the battery voltage for later broadcast.
    static void saveBatteryVoltage(double voltage) {
        int milliVolts = (int) (voltage * 1000);
        lastBatteryMilliVolts = milliVolts;
        minBatteryMilliVolts.getAndUpdate(v -> Math.min(v, milliVolts));
    }

    /// Record a begin/end bonus bracket for the current thread. All device calls made by the
    /// current thread while in a bracket are marked as "bonus" by the app. "Bonus" means that
    /// the user didn't call them - they were automatically invoked by either Sidekick or the
    /// system.
    void markBonus(boolean begin) {
        synchronized(sidekickLock) {
            if (Thread.currentThread().getId() != currentThreadId) {
                if (!threadSwitchAndBookkeeping())
                    return; // ====> Don't record any more
            }
            if ((chunkRemaining -= 2) < 0) {
                nextChunk();
            }
            chunk.putShort((short) (begin ? RecordId.BEGIN_BONUS : RecordId.END_BONUS));
            assertChunk();
        }
    }

    /// Record performance samples.
    void recordSamples(CpuSampler.Record cpuRecord, BandwidthSampler.Record storageRecord, MemoryAndThreadCountSampler.Record memoryRecord, SidekickSampler.Record sidekickRecord, short acquisitionTime) {
        synchronized(sidekickLock) {
            if (beginRecord(RecordId.SYSTEM_SAMPLES, CpuSampler.Record.PUT_SIZE + BandwidthSampler.Record.PUT_SIZE + MemoryAndThreadCountSampler.Record.PUT_SIZE + SidekickSampler.Record.PUT_SIZE + 2)) {
                cpuRecord.put(chunk);
                storageRecord.put(chunk);
                memoryRecord.put(chunk);
                sidekickRecord.put(chunk);
                chunk.putShort(acquisitionTime);
                assertChunk();
            }
        }
    }

    /// Record that [HardwareMap#get] (or a permutation) was called. Produces records that get
    /// reconstructed in the pattern of: hardwareMap.tryGet(DcMotor.class, "frontLeftMotor")
    void recordHardwareMapGet(String methodName, long nanoStartTime, Class<?> classOrInterface, String deviceName) {
        synchronized (sidekickLock) {
            RecordDescriptor recordDescriptor = hardwareMapDescriptors.get(methodName);
            if (recordDescriptor == null) {
                // We are recording a HardwareMap method so that's the type we specify here:
                DeviceInfo deviceInfo = hardwareMap.getDeviceInfo(HardwareMap.class);
                if (deviceInfo.deviceNames.isEmpty()) {
                    deviceInfo.deviceNames.add("HardwareMap");
                    // Nothing added to 'proxyInstance' because HardwareMap.get() will never be called
                }
                // Returns void:
                SchemaKey schemaKey = new SchemaKey(new byte[]{ParamType.STRING_NO_QUOTES, ParamType.STRING, ParamType.VOID});
                recordDescriptor = new RecordDescriptor(deviceInfo, methodName, schemaKey);
                hardwareMapDescriptors.put(methodName, recordDescriptor);
            }
            // There's only one HardwareMap so we don't need to specify an instance:
            if (beginRecord(recordDescriptor.recordId, 3, nanoStartTime)) {
                putInt24((int) ((nanoTime() - nanoStartTime) >> TICK_SHIFT));
                putString(classOrInterface.getSimpleName() + ".class");
                putString(deviceName);
                assertChunk();
            }
        }
    }

    /// Voltage sensor wrapper. We special case this wrapper rather than using Byte Buddy
    /// for 1) speed of class creation and 2) to avoid issues with invoking Byte Buddy while
    /// holding the Sidekick lock when we initialize the voltage sensor wrapper.
    class SkVoltageSensor implements VoltageSensor {
        VoltageSensor originalSensor; // Original sensor that we're wrapping
        int recordId;

        /// Constructor.
        SkVoltageSensor(SkHardwareMap hardwareMap, VoltageSensor originalSensor, String sensorName) {
            this.originalSensor = originalSensor;

            /// Do work to register the [RecordDescriptor] and allow [SkHardwareMap#wrapDevice] to
            /// return this device:
            DeviceInfo deviceInfo = hardwareMap.getDeviceInfo(originalSensor.getClass());
            int instance = deviceInfo.deviceNames.size();
            deviceInfo.deviceNames.add(sensorName);
            deviceInfo.proxyInstances.add(this); // Allow HardwareMap.get() to be called
            SchemaKey schemaKey = new SchemaKey(new byte[]{ParamType.FLOAT});
            RecordDescriptor recordDescriptor = new RecordDescriptor(deviceInfo, "getVoltage", schemaKey);
            recordId = recordDescriptor.recordId + instance;
        }

        /// Get the voltage from the original and record the result.
        @Override public double getVoltage() {
            synchronized (sidekickLock) {
                long nanoStartTime = nanoTime();
                double voltage = originalSensor.getVoltage();
                if (beginRecord(recordId, 7, nanoStartTime)) {
                    putInt24((int) ((nanoTime() - nanoStartTime) >> TICK_SHIFT));
                    chunk.putFloat((float) voltage);
                    assertChunk();
                }
                saveBatteryVoltage(voltage);
                return voltage;
            }
        }

        @Override public Manufacturer getManufacturer() { return originalSensor.getManufacturer(); }
        @Override public String getDeviceName() { return originalSensor.getDeviceName(); }
        @Override public String getConnectionInfo() { return originalSensor.getConnectionInfo(); }
        @Override public int getVersion() { return originalSensor.getVersion(); }
        @Override public void resetDeviceConfigurationForOpMode() { originalSensor.resetDeviceConfigurationForOpMode(); }
        @Override public void close() { originalSensor.close(); }
    }

    /// Record a global message reported by the system. `isError` is true if it's a global error
    /// message; false if it's a global warning message.
    void recordGlobalMessage(boolean isError, String message) {
        synchronized (sidekickLock) {
            if (beginRecord(RecordId.GLOBAL_MESSAGE, 1)) {
                chunk.put((byte) (isError ? 1 : 0));
                putString(message);
                assertChunk();
            }
        }
    }

    /// Record Lynx bulk data when a 'get' method is called that generates new bulk data.
    void recordLynxBulkRead(Capture capture, LynxModuleInfo lynxInfo) {
        synchronized (sidekickLock) {
            synchronized (lynxInfo.bulkCachingLock) {
                LynxModule.BulkData lastBulkData;
                LynxGetBulkInputDataResponse bulkResponse;

                try { // lastBulkData = lynxInfo.module.lastBulkData:
                    lastBulkData = (LynxModule.BulkData) capture.lynxModuleLastBulkDataField.get(lynxInfo.module);
                } catch (IllegalAccessException e) {
                    error(MinorError.BULK_READ_LYNX_MODULE);
                    return; // ====>
                }

                // If the bulk data has changed, record it:
                if ((lastBulkData != null) && (lastBulkData != lynxInfo.lastBulkData)) {
                    lynxInfo.lastBulkData = lastBulkData;

                    try { // bulkResponse = lastBulkData.resp:
                        bulkResponse = (LynxGetBulkInputDataResponse) capture.bulkDataRespField.get(lastBulkData);
                    } catch (IllegalAccessException e) {
                        error(MinorError.BULK_READ_LYNX_MODULE);
                        return; // ====>
                    }

                    // Obtain the bulk payload data from 'resp' and record it:
                    if (bulkResponse != null) {
                        int lynxIndex = lynxInfo.id;
                        byte[] currentPayload = bulkResponse.toPayloadByteArray();

                        // Bulk data is 34 bytes. To save space, we delta-encode from the previous results. We
                        // divide the buffer into a mixture of 8 integers and 2 bytes, with a 1-bit mask
                        // indicating which units changed. The first size computes the mask and the number of
                        // bytes needed for the record's payload.
                        byte[] lastPayload = lastLynxBulkPayload[lynxIndex];
                        int payloadSize = 2; // 2 bytes for header mask
                        int byteMask = 0; // 2 bits represents 2 bytes
                        int intMask = 0; // 8 bits represents 32 bytes
                        for (int i = 0; i < 2; i++) {
                            if (currentPayload[i] != lastPayload[i]) {
                                byteMask |= 1 << i;
                                payloadSize++;
                            }
                        }
                        IntBuffer current = ByteBuffer.wrap(currentPayload, 2, 32).asIntBuffer();
                        IntBuffer previous = ByteBuffer.wrap(lastPayload, 2, 32).asIntBuffer();
                        for (int i = 0; i < 8; i++) {
                            if (current.get(i) != previous.get(i)) {
                                intMask |= 1 << i;
                                payloadSize += 4;
                            }
                        }
                        if (beginRecord(RecordId.MODULE0_BULK_DATA + lynxIndex, payloadSize)) {
                            chunk.putShort((short) ((intMask << 2) | byteMask));
                            for (int i = 0; i < 2; i++) {
                                if ((byteMask & (1 << i)) != 0) {
                                    chunk.put(currentPayload[i]);
                                }
                            }
                            for (int i = 0; i < 8; i++) {
                                if ((intMask & (1 << i)) != 0) {
                                    chunk.putInt(current.get(i));
                                }
                            }
                            assertChunk();
                        }
                        lastLynxBulkPayload[lynxIndex] = currentPayload;
                    }
                }
            }
        }
    }

    /// Record bonus data when [GoBildaPinpointDriver#update] is called.
    void recordPinpointUpdate(GoBildaPinpointDriver wrappedPinpoint, int interceptorFlags) {
        if ((pinpointSetBulkReadScope) || ((interceptorFlags & Interceptor.Flags.PINPOINT_STATUS_UPDATE) != 0)) {
            // If the register scope has been set, don't record bulk data, only the status, as
            // they will presumably use all the registers they've set and so there won't be
            // any other 'bonus' data to read and report.
            markBonus(true);
            wrappedPinpoint.getDeviceStatus(); // Calling the wrapped object records the 'get' call
            markBonus(false);
        } else {
            synchronized(sidekickLock) {
                // The user hasn't set the register scope, so record all the data that
                // 'update()' provides:
                if (beginRecord(RecordId.PINPOINT_BULK_DATA, 38)) {
                    // Unwrap the Pinpoint object to avoid recording each individual 'get' call; we're
                    // batching the results together into one record here for better efficiency:
                    GoBildaPinpointDriver unwrapped = unwrap(wrappedPinpoint);
                    long startTime = nanoTime();
                    putEnumObject(unwrapped.getDeviceStatus());
                    chunk.putInt(unwrapped.getEncoderX());
                    chunk.putInt(unwrapped.getEncoderY());
                    chunk.putFloat((float) unwrapped.getPosX(DistanceUnit.INCH));
                    chunk.putFloat((float) unwrapped.getPosY(DistanceUnit.INCH));
                    chunk.putFloat((float) unwrapped.getHeading(AngleUnit.RADIANS));
                    chunk.putFloat((float) unwrapped.getVelX(DistanceUnit.INCH));
                    chunk.putFloat((float) unwrapped.getVelY(DistanceUnit.INCH));
                    chunk.putFloat((float) unwrapped.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
                    chunk.putShort((short) unwrapped.getLoopTime()); // Microseconds
                    short acquisitionMicroseconds = (short) ((nanoTime() - startTime) / 1000);
                    chunk.putShort(acquisitionMicroseconds);
                    assertChunk();
                }
            }
        }
    }

    ///----------------------------------------------------------------------------------------------
    /// The following serialize different types of method parameters that come in as objects.
    void putVoidObject(@Nullable Object param) {}
    void putIntegerObject(@Nullable Object param) {
        // Don't crash in the rare case of null from serializers or boxed method arguments:
        chunk.putInt(param == null ? 0 : ((Number) param).intValue());
    }
    void putFloatObject(@Nullable Object param) {
        // Don't crash in the rare case of null from serializers or boxed method arguments:
        chunk.putFloat(param == null ? 0 : ((Number) param).floatValue());
    }
    void putBooleanObject(@Nullable Object param) {
        // Don't crash in the rare case of null from serializers or boxed method arguments:
        chunk.put(param == null ? 0 : (byte) ((boolean) param ? 1 : 0)); // Byte 0 or 1
    }
    void putEnumObject(@Nullable Object param) {
        // It's legal in Java to pass 'null' to any Enum method argument. That's kind of odd.
        // The zero token is reserved for null:
        chunk.putShort(param == null ? 0 : tokenizeString(param.toString()));
    }
    void putStringObject(@Nullable Object param) {
        if (param == null) {
            putNullString();
        } else {
            putString(param.toString());
        }
    }
    void putToStringObject(@Nullable Object param) {
        // Some unrecognized objects will have nice implementations of 'toString()', but some
        // others will leave the default implementation. The default is long and ugly, e.g.:
        //      "com.qualcomm.hardware.sparkfun.SparkFunOTOS$Version@1b2543ea".
        // To save space and make it cleaner, when we see a default implementation, we shorten it:
        //      "SparkFunOTOS$Version"
        if (param == null) {
            putNullString();
        } else {
            if (param instanceof SidekickProxy) {
                param = ((SidekickProxy) param).unwrap(); // Use the real object if it was wrapped
            }
            String className = param.getClass().getName();
            String defaultToString = className + "@" + Integer.toHexString(System.identityHashCode(param));
            String toString = param.toString();
            putString(toString.equals(defaultToString) ? rootName(className) : toString);
        }
    }

    /// Save the string token table.
    void serializeStringTokens() {
        if ((chunkRemaining -= 6) < 0)
            nextChunk();
        chunk.putInt(Signature.STRING_TOKENS);
        chunk.putShort((short) stringTokensMap.size());
        for (String string: stringTokensList) {
            putString(string);
        }
        assertChunk();
    }

    /// Serialize the serializers.
    void serializeSerializers() {
        if ((chunkRemaining -= 6) < 0)
            nextChunk();
        chunk.putInt(Signature.SERIALIZER_DESCRIPTORS);
        chunk.putShort((short) sidekick.serializersMap.size());
        for (Sidekick.SerializerInfo<?> serializerInfo: sidekick.serializersList) {
            putString(serializerInfo.klass.getName());
            putString(serializerInfo.format);
        }
        assertChunk();
    }

    /// Save all of the record descriptors into chunks.
    void serializeRecordDescriptors() {
        // The first bytes of the section are for the signature and count of records:
        if ((chunkRemaining -= 6) < 0)
            nextChunk();
        chunk.putInt(Signature.RECORD_DESCRIPTORS);
        chunk.putShort((short) nextRecordId);
        assertChunk();

        // Now serialize every record:
        int expectedRecordId = RecordId.FIRST_ALLOCATABLE;
        for (RecordDescriptor descriptor: allocatedDescriptors) {
            assert(descriptor.recordId == expectedRecordId);
            expectedRecordId += descriptor.maxInstances;

            if ((chunkRemaining -= 3) < 0) {
                nextChunk();
            }
            chunk.put((byte) descriptor.deviceId); // Overflow check below
            chunk.put((byte) descriptor.childId);
            chunk.put((byte) descriptor.maxInstances);
            putString(descriptor.methodName);
            assertChunk();

            if ((chunkRemaining -= (1 + descriptor.formatBuffer.length)) < 0) {
                nextChunk();
            }
            chunk.put((byte) descriptor.formatBuffer.length); // Overflow check below
            chunk.put(descriptor.formatBuffer);
            assertChunk();

            // Overflow checks:
            if (descriptor.formatBuffer.length > 255) {
                error(MajorError.RECORD_FORMAT_TOO_LONG);
            }
            if (descriptor.deviceId > 255) {
                error(MajorError.DEVICE_ID_TOO_LARGE);
            }
        }
    }

    /// Save all of the device descriptors into chunks.
    void serializeDeviceDescriptors() {
        // The first bytes of the section are for the signature and count of records:
        if ((chunkRemaining -= 6) < 0)
            nextChunk();
        chunk.putInt(Signature.DEVICE_DESCRIPTORS);
        chunk.putShort((short) deviceClassArray.size());
        assertChunk();

        for (DeviceInfo classCache: deviceClassArray) {
            if ((chunkRemaining -= 3) < 0) {
                nextChunk();
            }
            chunk.put((byte) classCache.maxInstances);
            chunk.put((byte) classCache.busType.ordinal());
            chunk.put((byte) classCache.deviceNames.size());
            putString(getSdkName(classCache.originalClass));
            for (String deviceName: classCache.deviceNames) {
                putString(deviceName);
            }
            assertChunk();
        }
    }

    /// Save the Logcat output into chunks.
    void serializeLogcat() {
        File logcatFile = new File(logcatFilename);
        if (!logcatFile.exists()) {
            error(MinorError.LOGCAT);
        } else {
            if ((chunkRemaining -= 8) < 0)
                nextChunk();
            chunk.putInt(Signature.LOGCAT);
            chunk.putInt((int) logcatFile.length());
            assertChunk();
            try {
                try (FileInputStream inStream = new FileInputStream(logcatFile);
                     FileChannel in = inStream.getChannel()) {
                    while (in.read(chunk) >= 0) {
                        nextChunk();
                    }
                }
            } catch (IOException e) {
                // The capture file is corrupt if we couldn't fully read the file:
                error(MajorError.FILE_READ_ERROR);
            }
            chunkRemaining = chunk.remaining(); // Adjust for data we just added directly
            logcatFile.delete();
        }
    }

    /// Serialize the robot's configuration file.
    void serializeConfiguration() {
        String configName = null;
        try {
            Class<?> webInfoClass = Class.forName("org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo");
            Field configNameField = webInfoClass.getDeclaredField("cachedConfigName");
            configNameField.setAccessible(true);
            configName = (String) configNameField.get(null);
        } catch (ClassNotFoundException|NoSuchFieldException|IllegalAccessException ignored) {}

        if (configName != null) {
            // @@@ Move to Sidekick and read at Sidekick init for device names for sampling
            File configFile = new File(Sidekick.SD_CARD_PATH + "/FIRST/" + configName + ".xml");
            if (configFile.exists()) {
                if ((chunkRemaining -= 8) < 0)
                    nextChunk();
                chunk.putInt(Signature.CONFIGURATION);
                chunk.putInt((int) configFile.length());
                putString(configName);
                assertChunk();
                try {
                    try (FileInputStream inStream = new FileInputStream(configFile);
                         FileChannel in = inStream.getChannel()) {
                        while (in.read(chunk) >= 0) {
                            nextChunk();
                        }
                    }
                } catch (IOException e) {
                    // The capture file is corrupt if we couldn't fully read the file:
                    error(MajorError.FILE_READ_ERROR);
                }
                chunkRemaining = chunk.remaining(); // Adjust for data we just added directly
                return; // Success!
            }
        }
        error(MinorError.CONFIGURATION); // Failure!
    }

    /// Save all of the thread descriptors into chunks.
    void serializeThreadDescriptors() {
        if ((chunkRemaining -= 14) < 0)
            nextChunk();
        chunk.putInt(Signature.THREAD_DESCRIPTORS);
        chunk.putInt(processJiffyDuration);
        chunk.putInt(wallClockJiffyDuration);
        chunk.putShort((short) threadDescriptorList.size());
        assertChunk();

        for (ThreadDescriptor thread: threadDescriptorList) {
            if ((chunkRemaining -= 10) < 0) {
                nextChunk();
            }
            chunk.put((byte) (thread.isNew ? 1 : 0));
            chunk.putInt(thread.tid);
            chunk.putInt(thread.jiffyDuration);
            chunk.put((byte) thread.priority);
            putString(thread.name);
            assertChunk();
        }
    }

    /// Serialize the metadata manifest.
    void serializeManifest() {
        manifest.initUnixTimeMs = initDate.getTime(); // Unix time in milliseconds
        manifest.initNanoTime = initNanoTime;
        manifest.startNanoTime = startNanoTime;
        manifest.endNanoTime = endNanoTime;
        manifest.isExcessive = isExcessive;
        manifest.gamepadIsPs4 = gamepadIsPs4;
        manifest.suppressedIssues = sidekick.suppressedIssues.stream().mapToInt(Integer::intValue).toArray();
        manifest.appBuildTime = sidekick.appBuildTime;
        manifest.isBlocksOpMode = false; // @@@ opMode instanceof BlocksOpMode;
        manifest.availableProcessors = Runtime.getRuntime().availableProcessors();

        Class<? extends OpMode> klass = opMode.getClass();
        manifest.opModeClass = rootName(klass.getName());
        manifest.opModeName = manifest.opModeClass;
        TeleOp teleOp = klass.getAnnotation(TeleOp.class);
        Autonomous auto = klass.getAnnotation(Autonomous.class);
        if (teleOp != null) {
            if ((teleOp.name() != null) && !teleOp.name().isEmpty()) {
                manifest.opModeName = teleOp.name();
                if ((teleOp.group() != null) && !teleOp.group().isEmpty())
                    manifest.opModeName = teleOp.group() + ": " + manifest.opModeName;
            }
        } else if (auto != null) { // This null check is for unit tests
            if ((auto.name() != null) && !auto.name().isEmpty()) {
                manifest.opModeName = auto.name();
                if ((auto.group() != null) && !auto.group().isEmpty())
                    manifest.opModeName = auto.group() + ": " + manifest.opModeName;
            }
        }

        // Now serialize the resulting manifest. Can't call 'putString' because that limits the
        // string size.
        byte[] data = Sidekick.gson.toJson(manifest).getBytes(UTF8);
        if ((chunkRemaining -= 8) < 0)
            nextChunk();
        chunk.putInt(Signature.MANIFEST);
        chunk.putInt(data.length);
        putBytes(data);
    }

    /// Serialize privacy-sensitive data.
    void serializePii() {
        pii.robotName = ""; // @@@

//        InspectionState inspection = new InspectionState();
//        inspection.initializeLocal();
//        pii.robotName = inspection.deviceName;

        byte[] data = Sidekick.gson.toJson(pii).getBytes(UTF8);
        if ((chunkRemaining -= 8) < 0)
            nextChunk();
        chunk.putInt(Signature.PII);
        chunk.putInt(data.length);
        putBytes(data);
    }

    /// Initialize Lynx module information for Bulk Read processing.
    void initializeLynxModuleInfo() {
        try {
            bulkDataRespField = LynxModule.BulkData.class.getDeclaredField("resp");
            bulkDataRespField.setAccessible(true);
            lynxModuleLastBulkDataField = LynxModule.class.getDeclaredField("lastBulkData");
            lynxModuleLastBulkDataField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            error(MinorError.BULK_READ_LYNX_MODULE);
            return; // ====>
        }

        // Be sure to use the original hardware map to avoid our 'getAll()' wrap!
        List<LynxModule> lynxModuleList = hardwareMap.originalMap.getAll(LynxModule.class);
        if (lynxModuleList.isEmpty()) {
            error(MinorError.BULK_READ_LYNX_MODULE);
        }
        // 'lynxModuleMap' does not have to be complete in the event of failure:
        manifest.lynxAddresses = new int[lynxModuleList.size()];
        for (LynxModule module: lynxModuleList) {
            Object bulkCachingLock = getField(module, "bulkCachingLock", Object.class);
            if ((bulkCachingLock != null) && (lynxModuleMap.size() < MAX_LYNX_COUNT)) {
                LynxModuleInfo lynxInfo = new LynxModuleInfo(bulkCachingLock);
                lynxInfo.module = module;
                lynxInfo.id = lynxModuleMap.size();
                manifest.lynxAddresses[lynxInfo.id] = module.getModuleAddress();
                lynxModuleMap.put(module, lynxInfo);
            } else {
                error(MinorError.BULK_READ_LYNX_MODULE, "Couldn't prepare for bulk reads");
            }
        }
    }

    /// Delete the oldest captures until there's sufficient space for a new capture.
    void clearSpaceForNewCapture() {
        StatFs stat = new StatFs(Sidekick.SD_CARD_PATH);

        Sidekick.logI("^^^ Available space: %,d bytes", stat.getAvailableBytes());

        File directory = new File(Sidekick.SUBDIRECTORY);
        File[] sidekickFiles = directory.listFiles((d, name) ->
                name.endsWith(".sidekick")
        );
        if (sidekickFiles != null) {
            Arrays.sort(sidekickFiles, (a, b) ->
                    a.getName().compareToIgnoreCase(b.getName())
            );

            // Delete all Sidekick files that start with "1900-". Sidekick captures will be
            // created with that year only when OpModes are launched without having a connected
            // Driver Station to supply the robot with the current date and time. (I think
            // this can happen only with FTC Dashboard.) Multiple files with "1900-" mess
            // up our logic for downloading the newest capture so we contrive to ensure that
            // there is a maximum of one such file:
            for (File f: sidekickFiles) {
                if (f.getName().startsWith("1900-")) {
                    Sidekick.logI("Deleting " + f.getName());
                    f.delete();
                } else {
                    break; // ====>
                }
            }

            // Now delete the oldest files until there's sufficient space:
            for (File f: sidekickFiles) {
                if (stat.getAvailableBytes() >= MIN_STORAGE_MBS*1024*1024)
                    break; // ====>
                Sidekick.logI("Deleting " + f.getName());
                f.delete();
            }
        }
    }

    /// Initialize to begin capturing.
    void initializeCapture(OpModeManagerImpl opModeManager, boolean isGamepadHooked) {
        assert(Thread.holdsLock(sidekickLock));
        clearSpaceForNewCapture();
        if (!isGamepadHooked) {
            error(MinorError.GAMEPAD);
        }

        // Initialize the file and allocate a chunk for capturing, write the file header, and
        // initialize the archive section:
        sidekick.fileWorker.submit(new FileWorker.OpenMessage(dateAndTime));
        chunk = sidekick.fileWorker.getChunk();
        chunk.put(Header.create(0, 0, startUnixTime, sections));
        sections[Section.ARCHIVE].offset = getOffset();
        chunk.putInt(Signature.ARCHIVE_DATA);
        chunkRemaining = chunk.remaining();

        // Replace hardwareMap with our own implementation:
        if (!(opMode.hardwareMap instanceof SkHardwareMap)) {
            hardwareMap = new SkHardwareMap(opMode.hardwareMap, this, opModeManager);
            opMode.hardwareMap = hardwareMap;
        }

        // Wrap telemetry with our proxy:
        if (!(opMode.telemetry instanceof Capture.SkTelemetry)) {
            telemetry = new SkTelemetry(opMode.telemetry);
            opMode.telemetry = telemetry;
        }

        // Lynx Module info is used for Bulk Read processing:
        initializeLynxModuleInfo();

        // Decide on the filename for capturing Logcat data and enlighten the emulator:
        logcatFilename = Sidekick.SUBDIRECTORY + "/" + dateAndTime + ".logcat";
        if (sidekick.emulatorCallback != null) {
            sidekick.emulatorCallback.apply(0, logcatFilename);
        } else {
            // Start a process that will save Logcat output to a file while the opMode runs:
            try {
                logcatProcess = new ProcessBuilder(
                        "logcat",
                        "-v", "epoch", // Format with Unix timestamps
                        "-v", "usec", // Format with microseconds
                        "-f", logcatFilename, // Save to our file
                        "*:V" // All tags, full verboseness
                ).redirectErrorStream(true).start();
            } catch (IOException e) {
                error(MinorError.LOGCAT);
            }
        }
        Sidekick.logI("INIT was pressed"); // Mark the start of an opMode
    }

    /// Stop capturing and save all of the metadata.
    void endCapture() {
        synchronized (sidekickLock) {
            assert(!isEnded);

            // Stop Limelight recording:
            if (limelightCapture != null) {
                limelightCapture.stop();
                if (limelightCapture.hasError) {
                    error(MinorError.LIMELIGHT_VIDEO_CAPTURE);
                }
            }

            // Stop screen recording:
            Socket.broadcastStopScreenRecord();

            // If the opMode terminated because of an unhandled exception, record it:
            RuntimeException exception = getField(opMode,"exception", RuntimeException.class);
            if (exception != null) {
                if (beginRecord(RecordId.UNHANDLED_EXCEPTION, 0)) {
                    StringWriter writer = new StringWriter();
                    exception.printStackTrace(new PrintWriter(writer));

                    // Record the exception type, the optional message, and the stack trace:
                    putString(exception.getClass().getName());
                    putString(exception.getMessage() != null ? exception.getMessage() : "");
                    putString(writer.toString(), 4096);
                }
            }

            // Finish off the Archive section:
            endNanoTime = nanoTime();
            if ((chunkRemaining -= 5) < 0)
                nextChunk();
            chunk.putShort((short) RecordId.END_ARCHIVE);
            putInt24((int) endNanoTime);
            sections[Section.ARCHIVE].size = getOffset() - sections[Section.ARCHIVE].offset;

            // Snapshot the current thread state:
            snapshotThreadsAndJiffies(false);

            // Terminate Logcat logging and then serialize to the Logcat section:
            sections[Section.LOGCAT].offset = getOffset();
            if (logcatProcess != null) {
                logcatProcess.destroy();
                try {
                    logcatProcess.waitFor();
                } catch (InterruptedException ignored) {}
            }
            serializeLogcat();
            sections[Section.LOGCAT].size = getOffset() - sections[Section.LOGCAT].offset;

            // Serialize the Metadata section:
            sections[Section.METADATA].offset = getOffset();
            serializeManifest();
            serializeConfiguration();
            serializeDeviceDescriptors();
            serializeStringTokens();
            serializeSerializers();
            serializeRecordDescriptors(); // Must come last
            serializeThreadDescriptors();
            sections[Section.METADATA].size = getOffset() - sections[Section.METADATA].offset;

            // PII gets its own section:
            sections[Section.PII].offset = getOffset();
            serializePii();
            sections[Section.PII].size = getOffset() - sections[Section.PII].offset;

            // Queue the last chunk and then close the file:
            CountDownLatch limelightDoneLatch = limelightCapture != null ? limelightCapture.doneLatch : null;
            sidekick.fileWorker.submit(new FileWorker.WriteMessage(chunk));
            sidekick.fileWorker.submit(new FileWorker.CloseMessage(majorErrors, minorErrors, sections,
                    opMode.getClass(), dateAndTime, startUnixTime, startNanoTime, endNanoTime,
                    sidekick.lastRunNumber, limelightDoneLatch, fileDoneLatch));

            // Disable any further recording on this capture context:
            currentThreadId = 0; /// Set to zero to force [#threadSwitchAndBookkeeping] calls
            isEnded = true;
        }
    }

    ///---------------------------------------------------------------------------------------------
    /// API entry points below here

    /// Return the original delegate of a wrapped proxy object.
    static <T> T unwrap(T device) {
        if (device instanceof SidekickProxy) {
            return (T) ((SidekickProxy) device).unwrap();
        } else if (device instanceof SkHardwareMap) {
            return (T) ((SkHardwareMap) device).originalMap;
        } else if (device instanceof SkTelemetry) {
            return (T) ((SkTelemetry) device).originalTelemetry;
        } else if (device instanceof SkVoltageSensor) {
            return (T) ((SkVoltageSensor) device).originalSensor;
        } else {
            return device;
        }
    }

    /// Add the contents of an FTC Dashboard canvas to the capture.
    void captureCanvas(Object canvas) {
        recordPaint(new Paint(canvas), CanvasFlags.ROTATED_FIELD);
    }
    void recordInferredDashboardCanvas(Object canvas) {
        recordPaint(new Paint(canvas), CanvasFlags.ROTATED_FIELD | CanvasFlags.INFERRED);
    }

    /// Add data from a gamepad input packet to the capture, modeled off [EventLoopManager#gamepadEvent].
    void gamepadData(byte[] input) {
        final int GAMEPAD_FLOAT_COUNT = 6; // Gamepad data always has 6 floats

        synchronized (sidekickLock) {
            if (isEnded)
                return; // ====>

            // Decode:
            final int headerOffset = 5;
            ByteBuffer inBuffer = ByteBuffer.wrap(input, headerOffset, input.length - headerOffset);
            byte version = inBuffer.get();
            if (version < 3) {
                error(MinorError.GAMEPAD);
                return; // ====>
            }

            int ignoredId = inBuffer.getInt();
            long timestamp = inBuffer.getLong(); // SystemClock.uptimeMillis()
            // Collect all of the floats, creating a bitmask to mark the non-zero values:
            float[] floats = new float[GAMEPAD_FLOAT_COUNT];
            int nonzeroMask = 0;
            int payloadSize = 0;
            for (int i = 0; i < GAMEPAD_FLOAT_COUNT; i++) {
                floats[i] = inBuffer.getFloat();
                if (floats[i] != 0.0) {
                    nonzeroMask |= 1 << i;
                    payloadSize += 4;
                }
            }
            int buttons = inBuffer.getInt();
            if (buttons != 0) {
                nonzeroMask |= 1 << GAMEPAD_FLOAT_COUNT;
                payloadSize += 2;
            }
            byte gamepadUser = inBuffer.get(); // GamepadUser
            byte legacyType = inBuffer.get(); // LegacyType

            if ((gamepadUser < 1) || (gamepadUser > 2)) {
                error(MinorError.GAMEPAD);
                return; // ====>
            }

            // Remember the gamepad type but don't bother saving it with the gamepad data:
            gamepadIsPs4[gamepadUser - 1] = (legacyType == Gamepad.LegacyType.SONY_PS4.ordinal());

            // Write the capture record without recording the current thread's ID as it will
            // always be a system thread.
            if ((chunkRemaining -= (8 + payloadSize)) < 0) {
                nextChunk();
            }
            chunk.putShort((short) ((gamepadUser == 1) ? RecordId.GAMEPAD1_DATA : RecordId.GAMEPAD2_DATA));
            long tickTime = nanoTime() >> TICK_SHIFT;
            chunk.put((byte) (tickTime >> 16));
            chunk.putShort((short) tickTime);
            chunk.putShort((short) timestamp);

            // To save space, don't bother writing values that are zero:
            chunk.put((byte) nonzeroMask);
            for (int i = 0; i < GAMEPAD_FLOAT_COUNT; i++) {
                if (floats[i] != 0.0) {
                    chunk.putFloat(floats[i]);
                }
            }
            if (buttons != 0) {
                chunk.putShort((short) buttons);
            }
            assertChunk();
        }
    }

    /// Add the contents of a [Paint] object to the capture.
    void recordPaint(Paint paint) { recordPaint(paint, 0); }
    void recordPaint(Paint paint, int flags) {
        synchronized (sidekickLock) {
            if (beginRecord(RecordId.PAINT, 3)) {
                ByteBuffer buffer = paint.buffer;
                int size = buffer.position();

                chunk.put((byte) flags);
                chunk.putShort((short) size);
                putBytes(paint.buffer.array(), size);
                assertChunk();
            }
        }
    }

    /// Mark the beginning/end of a loop.
    void loop(Sk.LoopType loopType) {
        synchronized (sidekickLock) {
            if (beginRecord(RecordId.LOOP, 1)) {
                chunk.put((byte) loopType.ordinal());
                assertChunk();
            }
        }
    }

    /// API to record a note into the archive. This is performance critical so it's largely
    /// inlined and with a special case carved off for zero arguments.
    void note(String format, Object... args) {
        synchronized (sidekickLock) {
            if (args.length == 0) {
                if (!beginRecord(RecordId.NOTE_STRING, 0))
                    return; // ====>
                putString(format); // We assume the string is dynamic so don't tokenize
            } else {
                if (Thread.currentThread().getId() != currentThreadId) {
                    if (!threadSwitchAndBookkeeping())
                        return; // ====>
                }
                SchemaKey schemaKey = getSchemaKey(args, false, 0);
                RecordDescriptor descriptor = varargsDescriptors.get(schemaKey);
                if (descriptor == null) {
                    descriptor = new RecordDescriptor(schemaKey, args.length);
                }
                if ((chunkRemaining -= (7 + descriptor.recordLength)) < 0) {
                    nextChunk();
                }
                chunk.putShort((short) (descriptor.recordId + VarargsType.NOTE));
                long tickTime = nanoTime() >> TICK_SHIFT;
                chunk.put((byte) (tickTime >> 16));
                chunk.putShort((short) tickTime);
                chunk.putShort(tokenizeString(format)); // Assume it's not a dynamic string so tokenize
                for (int i = 0; i < descriptor.paramPutters.length; i++) {
                    descriptor.paramPutters[i].accept(args[i]);
                }
            }
            assertChunk();
        }
    }

    /// Helper for recording [Sk#alert] and [Sk#require] varargs records. These are not
    /// as performance critical as [Sk#note] so they employ a common helper function.
    void recordVarargs(int varargsType, String format, Object... args) {
        synchronized(sidekickLock) {
            SchemaKey schemaKey = getSchemaKey(args, false, 0);
            RecordDescriptor descriptor = varargsDescriptors.get(schemaKey);
            if (descriptor == null) {
                descriptor = new RecordDescriptor(schemaKey, args.length);
            }
            if (beginRecord(descriptor.recordId + varargsType,
                    descriptor.recordLength + 2)) {
                chunk.putShort(tokenizeString(format)); // Assume it's not a dynamic string so tokenize
                for (int i = 0; i < descriptor.paramPutters.length; i++) {
                    descriptor.paramPutters[i].accept(args[i]);
                }
                assertChunk();

                // Requirements add a string that is the current stack trace:
                if (varargsType == VarargsType.REQUIRE) {
                    StackTraceElement[] elements = Thread.currentThread().getStackTrace();
                    StringBuilder sb = new StringBuilder(256);
                    // Skip: [0] getStackTrace, [1] current(), [2] caller of current()
                    for (int i = 3; i < elements.length; i++) {
                        sb.append(elements[i]).append('\n');
                    }
                    putString(sb.toString());
                }
            }
        }
    }

    /// We wrap the [Telemetry] object with a custom implementation - no Byte Buddy here.
    /// The FTC reference implementation can be found in [TelemetryImpl].
    public class SkTelemetry implements Telemetry, TelemetryInternal {
        final private Telemetry originalTelemetry; // Original telemetry object
        final private SkLog wrapperLog; // Wrapped log object

        /// Wrap the opMode's telemetry object.
        public SkTelemetry(Telemetry original) {
            this.originalTelemetry = original;
            this.wrapperLog = new SkLog(original.log());
        }

        /// After the original call is made, this logs the call and creates the wrapper Item.
        SkItem telemetryAddData(String caption, String format, Item originalItem, int lineId, Object... args) {
            synchronized (sidekickLock) {
                SkItem wrappedItem = new SkItem(originalItem, lineId); // Must be under lock
                SchemaKey schemaKey = getSchemaKey(args, false, 0);
                RecordDescriptor descriptor = varargsDescriptors.get(schemaKey);
                if (descriptor == null) {
                    descriptor = new RecordDescriptor(schemaKey, args.length);
                }
                byte flags = 0;
                int payload = 6; // byte + short + int24
                if (lineId != 0) { // lineId is 0 for the common case of 'telemetry.addData(...)'
                    flags |= VarargsFlags.ADD_DATA_WITH_LINE_ID;
                    payload += 3; // int24
                }
                if (format != null) {
                    flags |= VarargsFlags.ADD_DATA_WITH_FORMAT;
                    payload += 2; // short
                }
                if (!beginRecord(descriptor.recordId + VarargsType.TELEMETRY_ADD_DATA,
                        payload + descriptor.recordLength))
                    return wrappedItem; // ====>

                chunk.putShort(tokenizeString(caption));
                putInt24(wrappedItem.itemId);
                chunk.put(flags);
                if ((flags & VarargsFlags.ADD_DATA_WITH_LINE_ID) != 0) {
                    putInt24(lineId);
                }
                if ((flags & VarargsFlags.ADD_DATA_WITH_FORMAT) != 0) {
                    chunk.putShort(tokenizeString(format));
                }
                for (int i = 0; i < descriptor.paramPutters.length; i++) {
                    descriptor.paramPutters[i].accept(args[i]);
                }
                assertChunk();
                return wrappedItem;
            }
        }

        /// After the original call is made, this logs the call and creates the wrapper Item.
        /// Flags is SET_VALUE_CONSUMER_UPDATE if update happens from a consumer callback.
        SkItem telemetrySetValue(int flags, SkItem wrappedItem, String format, Object... args) {
            synchronized (sidekickLock) {
                SchemaKey schemaKey = getSchemaKey(args, false, 0);
                RecordDescriptor descriptor = varargsDescriptors.get(schemaKey);
                if (descriptor == null) {
                    descriptor = new RecordDescriptor(schemaKey, args.length);
                }

                int payload = 4; // byte + int24
                if (format != null) {
                    flags |= VarargsFlags.SET_VALUE_WITH_FORMAT;
                    payload += 2; // short
                }
                if (!beginRecord(descriptor.recordId + VarargsType.TELEMETRY_SET_VALUE,
                        payload + descriptor.recordLength))
                    return wrappedItem; // ====>

                putInt24(wrappedItem.itemId);
                chunk.put((byte) flags);
                if ((flags & VarargsFlags.SET_VALUE_WITH_FORMAT) != 0) {
                    chunk.putShort(tokenizeString(format));
                }
                for (int i = 0; i < descriptor.paramPutters.length; i++) {
                    descriptor.paramPutters[i].accept(args[i]);
                }
                assertChunk();
                return wrappedItem;
            }
        }

        /// Wrap and capture producer arguments to [Telemetry#addData] and [Telemetry.Item#setValue]
        class ProducerWrapper<T> implements Func<T> {
            Func<T> originalProducer;
            String format; // Can be null
            SkItem wrappedItem;

            // Format can be null.
            ProducerWrapper(Func<T> originalProducer, String format) {
                this.originalProducer = originalProducer;
                this.format = format;
            }

            // Register the wrapped item associated with this producer.
            void setWrapperItem(SkItem wrappedItem) {
                this.wrappedItem = wrappedItem;
            }

            // This is called when the Telemetry log wants the app to generate the value. We
            // translate this into a setValue() call for the archive.
            @Override public T value() {
                T value = originalProducer.value();
                // Log the value the callback generated:
                telemetrySetValue(VarargsFlags.SET_VALUE_PRODUCER_UPDATE, wrappedItem, format, value);
                return value;
            }
        }

        /// Wrap a [Telemetry.Item] object for capture purposes.
        public class SkItem implements Item {
            final private Item originalItem; // Original item object that we're wrapping
            final private int itemId; // Sidekick record identifier
            final private int lineId; // 0 if no parent; parent line's ID otherwise

            // lineId is 0 if no parent, otherwise it's the parent line's ID
            SkItem(Item originalItem, int lineId) {
                this.originalItem = originalItem;
                this.itemId = generateUniqueness();
                this.lineId = lineId;
            }

            @Override public SkItem setValue(String format, Object... args) {
                originalItem.setValue(format, args);
                return telemetrySetValue(0, this, format, args);
            }

            @Override public SkItem setValue(Object value) {
                originalItem.setValue(value);
                return telemetrySetValue(0, this, null, value);
            }

            @Override public <T> SkItem setValue(Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, null);
                originalItem.setValue(producerWrapper);
                telemetrySetValue(0, this, null); // Empty set indicates a producer
                producerWrapper.setWrapperItem(this);
                return this;
            }

            @Override public <T> SkItem setValue(String format, Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, format);
                originalItem.setValue(producerWrapper);
                telemetrySetValue(0, this, null); // Empty set indicates a producer
                producerWrapper.setWrapperItem(this);
                return this;
            }

            @Override public SkItem setCaption(String caption) {
                originalItem.setCaption(caption);
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.TELEMETRY_SET_CAPTION, 5)) {
                        putInt24(itemId);
                        chunk.putShort(tokenizeString(caption));
                        assertChunk();
                    }
                }
                return this;
            }

            @Override public SkItem setRetained(@Nullable Boolean retained) {
                originalItem.setRetained(retained);
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.TELEMETRY_SET_RETAINED, 4)) {
                        putInt24(itemId);
                        chunk.put((byte) (retained == null ? 0 : (retained ? 1 : 0)));
                        assertChunk();
                    }
                }
                return this;
            }

            @Override public SkItem addData(String caption, String format, Object... args) {
                Item originalItem = this.originalItem.addData(caption, format, args);
                return telemetryAddData(caption, format, originalItem, lineId, args);
            }

            @Override public SkItem addData(String caption, Object value) {
                Item originalItem = this.originalItem.addData(caption, value);
                return telemetryAddData(caption, null, originalItem, lineId, value);
            }

            @Override public <T> SkItem addData(String caption, Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, null);
                Item originalItem = this.originalItem.addData(caption, producerWrapper);
                SkItem wrappedItem = telemetryAddData(caption, null, originalItem, lineId);
                producerWrapper.setWrapperItem(wrappedItem);
                return wrappedItem;
            }

            @Override public <T> SkItem addData(String caption, String format, Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, format);
                Item originalItem = this.originalItem.addData(caption, format, producerWrapper);
                SkItem wrappedItem = telemetryAddData(caption, format, originalItem, lineId);
                producerWrapper.setWrapperItem(wrappedItem);
                return wrappedItem;
            }

            /// Not captured:
            @Override public String getCaption() { return originalItem.getCaption(); }
            @Override public boolean isRetained() { return originalItem.isRetained(); }
        }

        /// Wrap a [Telemetry.Line] object for capture purposes.
        public class SkLine implements Line {
            private final Line originalLine; // Original line object that we're wrapping
            private final int identifier; // Unique identifier for this object

            SkLine(Line original) {
                this.originalLine = original;
                this.identifier = generateUniqueness();
            }

            @Override public SkItem addData(String caption, String format, Object... args) {
                Item originalItem = originalLine.addData(caption, format, args);
                return telemetryAddData(caption, format, originalItem, identifier, args);
            }

            @Override public SkItem addData(String caption, Object value) {
                Item originalItem = originalLine.addData(caption, value);
                return telemetryAddData(caption, null, originalItem, identifier, value);
            }

            @Override public <T> SkItem addData(String caption, Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, null);
                Item originalItem = originalLine.addData(caption, producerWrapper);
                SkItem wrappedItem = telemetryAddData(caption, null, originalItem, identifier);
                producerWrapper.setWrapperItem(wrappedItem);
                return wrappedItem;
            }

            @Override public <T> SkItem addData(String caption, String format, Func<T> valueProducer) {
                ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, format);
                Item originalItem = originalLine.addData(caption, format, producerWrapper);
                SkItem wrappedItem = telemetryAddData(caption, format, originalItem, identifier);
                producerWrapper.setWrapperItem(wrappedItem);
                return wrappedItem;
            }
        }

        /// Wrap a [Telemetry.Log] object for capture purposes.
        protected class SkLog implements Log {
            private final Log originalLog;

            SkLog(Log originalLog) {
                this.originalLog = originalLog;
            }

            @Override public void setCapacity(int capacity) {
                originalLog.setCapacity(capacity);
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.LOG_SET_CAPACITY, 4)) {
                        chunk.putShort((short) capacity);
                        assertChunk();
                    }
                }
            }

            @Override public void setDisplayOrder(DisplayOrder displayOrder) {
                originalLog.setDisplayOrder(displayOrder);
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.LOG_SET_DISPLAY_ORDER, 1)) {
                        chunk.put((byte) displayOrder.ordinal());
                        assertChunk();
                    }
                }
            }

            @Override public void add(String entry) {
                originalLog.add(entry);
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.LOG_ADD_STRING, 0)) {
                        putString(entry);
                        assertChunk();
                    }
                }
            }

            @Override public void add(String format, Object... args) {
                originalLog.add(format, args);
                synchronized (sidekickLock) {
                    SchemaKey schemaKey = getSchemaKey(args, false, 0);
                    RecordDescriptor descriptor = varargsDescriptors.get(schemaKey);
                    if (descriptor == null) {
                        descriptor = new RecordDescriptor(schemaKey, args.length);
                    }

                    if (beginRecord(descriptor.recordId + VarargsType.LOG_ADD,
                            2 + descriptor.recordLength)) {
                        chunk.putShort(tokenizeString(format));
                        for (int i = 0; i < descriptor.paramPutters.length; i++) {
                            descriptor.paramPutters[i].accept(args[i]);
                        }
                        assertChunk();
                    }
                }
            }

            @Override public void clear() {
                originalLog.clear();
                synchronized (sidekickLock) {
                    if (beginRecord(RecordId.LOG_CLEAR, 0)) {
                        assertChunk();
                    }
                }
            }

            /// Not captured:
            @Override public int getCapacity() { return originalLog.getCapacity(); }
            @Override public DisplayOrder getDisplayOrder() { return null; }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////
        /// Telemetry methods
        @Override public SkItem addData(String caption, String format, Object... args) {
            Item originalItem = originalTelemetry.addData(caption, format, args);
            return telemetryAddData(caption, format, originalItem, 0, args);
        }

        @Override public SkItem addData(String caption, Object value) {
            Item originalItem = originalTelemetry.addData(caption, value);
            return telemetryAddData(caption, null, originalItem, 0, value);
        }

        /// We capture producers as [Telemetry.Item#setValue] calls.
        @Override public <T> SkItem addData(String caption, Func<T> valueProducer) {
            ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, null);
            Item originalItem = originalTelemetry.addData(caption, producerWrapper);
            SkItem wrappedItem = telemetryAddData(caption, null, originalItem, 0);
            producerWrapper.setWrapperItem(wrappedItem);
            return wrappedItem;
        }

        /// We capture producers as [Telemetry.Item#setValue] calls.
        @Override public <T> SkItem addData(String caption, String format, Func<T> valueProducer) {
            ProducerWrapper<T> producerWrapper = new ProducerWrapper<>(valueProducer, format);
            Item originalItem = originalTelemetry.addData(caption, format, producerWrapper);
            SkItem wrappedItem = telemetryAddData(caption, format, originalItem, 0);
            producerWrapper.setWrapperItem(wrappedItem);
            return wrappedItem;
        }

        @Override public boolean removeItem(Item item) {
            synchronized (sidekickLock) {
                SkItem wrappedItem = (SkItem) item;
                if (beginRecord(RecordId.TELEMETRY_REMOVE_ITEM, 3)) {
                    putInt24(wrappedItem.itemId);
                    assertChunk();
                }
            }
            return originalTelemetry.removeItem(item);
        }

        @Override public void clear() {
            originalTelemetry.clear();
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_CLEAR, 0)) {
                    assertChunk();
                }
            }
        }

        @Override public void clearAll() {
            originalTelemetry.clearAll();
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_CLEAR_ALL, 0)) {
                    assertChunk();
                }
            }
        }

        @Override public void speak(String text) {
            originalTelemetry.speak(text);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SPEAK, 0)) {
                    putString(text);
                    assertChunk();
                }
            }
        }

        @Override public void speak(String text, String languageCode, String countryCode) {
            originalTelemetry.speak(text, languageCode, countryCode);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SPEAK, 0)) {
                    putString(text);
                    assertChunk();
                }
            }
        }

        @Override public boolean update() {
            boolean result = originalTelemetry.update();
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_UPDATE, 1)) {
                    chunk.put((byte) (result ? 1 : 0));
                    assertChunk();
                }
            }
            return result;
        }

        @Override public SkLine addLine() { return addLine(""); }
        @Override public SkLine addLine(String lineCaption) {
            SkLine wrappedLine = new SkLine(originalTelemetry.addLine(lineCaption));
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_ADD_LINE, 3)) {
                    putInt24(wrappedLine.identifier);
                    putString(lineCaption, 16*1024); // Allow really big strings
                    assertChunk();
                }
            }
            return wrappedLine;
        }

        @Override public boolean removeLine(Line line) {
            SkLine wrappedLine = (SkLine) line;
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_REMOVE_LINE, 3)) {
                    putInt24(wrappedLine.identifier);
                    assertChunk();
                }
            }
            return originalTelemetry.removeLine(line);
        }

        @Override public void setAutoClear(boolean autoClear) {
            originalTelemetry.setAutoClear(autoClear);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SET_AUTO_CLEAR, 0)) {
                    assertChunk();
                }
            }
        }

        @Override public void setItemSeparator(String itemSeparator) {
            originalTelemetry.setItemSeparator(itemSeparator);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SET_ITEM_SEPARATOR, 0)) {
                    putString(itemSeparator);
                    assertChunk();
                }
            }
        }

        @Override public void setCaptionValueSeparator(String captionValueSeparator) {
            originalTelemetry.setCaptionValueSeparator(captionValueSeparator);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SET_CAPTION_VALUE_SEPARATOR, 0)) {
                    putString(captionValueSeparator);
                    assertChunk();
                }
            }
        }

        @Override public void setDisplayFormat(DisplayFormat displayFormat) {
            originalTelemetry.setDisplayFormat(displayFormat);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SET_DISPLAY_FORMAT, 1)) {
                    chunk.put((byte) displayFormat.ordinal());
                    assertChunk();
                }
            }
        }

        @Override public void setMsTransmissionInterval(int msTransmissionInterval) {
            originalTelemetry.setMsTransmissionInterval(msTransmissionInterval);
            synchronized (sidekickLock) {
                if (beginRecord(RecordId.TELEMETRY_SET_MS_TRANSMISSION_INTERVAL, 2)) {
                    chunk.putShort((short) msTransmissionInterval);
                    assertChunk();
                }
            }
        }

        // Not captured:
        @Override public boolean isAutoClear() { return originalTelemetry.isAutoClear(); }
        @Override public int getMsTransmissionInterval() { return originalTelemetry.getMsTransmissionInterval(); }
        @Override public String getItemSeparator() { return originalTelemetry.getItemSeparator(); }
        @Override public String getCaptionValueSeparator() { return originalTelemetry.getCaptionValueSeparator(); }
        @Override public Object addAction(Runnable action) { return originalTelemetry.addAction(action); }
        @Override public boolean removeAction(Object token) { return originalTelemetry.removeAction(token); }
        @Override public Log log() { return this.wrapperLog; }
        @Override public boolean tryUpdateIfDirty() { return ((TelemetryInternal)originalTelemetry).tryUpdateIfDirty(); }
        @Override public void resetTelemetryForOpMode() { ((TelemetryInternal)originalTelemetry).resetTelemetryForOpMode(); }
    }

    /// SkHardwareMap doesn't wrap the original [HardwareMap] instantiation but rather replaces it
    /// entirely. However, it shares all of the public and protected fields of the original.
    @SuppressWarnings("unchecked")
    public class SkHardwareMap extends HardwareMap {
        HardwareMap originalMap; // Original unhooked HardwareMap
        Capture capture; // Associated capture

        SkHardwareMap(HardwareMap original, Capture capture, OpModeManagerNotifier notifier) {
            super(original.appContext, notifier);
            this.originalMap = original;
            this.capture = capture;

            // Copy all of the public and protected fields from the original HardwareMap:
            Class<?> clazz = original.getClass();
            for (Field field : clazz.getDeclaredFields()) {  // scan all fields
                int modifiers = field.getModifiers();
                if (!Modifier.isFinal(modifiers) && !Modifier.isPrivate(modifiers)) { // Can't overwrite final fields
                    try {
                        field.setAccessible(true); // Make protected fields accessible
                        Object value = field.get(original);
                        field.set(this, value);
                    } catch (IllegalAccessException e) {
                        error(MinorError.HARDWARE_MAP, "*** HardwareMap failure!");
                    }
                }
            }

            /// We normally wrap hardware devices with our Sidekick instrumented proxies when
            /// [HardwareMap#get] is called. However, [FtcEventLoopHandler#buildRobotBatteryMsg]
            /// and libraries such as Road Runner bypass get() and directly access the
            /// [HardwareMap#voltageSensor] [DeviceMapping] in order to acquire voltage sensor
            /// instances. As such, we wrap the contents of that DeviceMapping here. Note that
            /// [Sidekick#onOpModePreInit] is guaranteed to have already primed the cache for
            /// VoltageSensor outside of the Sidekick lock.
            Map<String, VoltageSensor> map = null;
            try {
                Class<?> deviceMappingClass = original.voltageSensor.getClass();
                Field mapField = deviceMappingClass.getDeclaredField("map");
                mapField.setAccessible(true);
                map = (Map<String, VoltageSensor>) (mapField.get(original.voltageSensor));
            } catch (NoSuchFieldException | IllegalAccessException ignored) {}

            if (map != null) {
                for (Map.Entry<String, VoltageSensor> entry : map.entrySet()) {
                    entry.setValue(new SkVoltageSensor(this, entry.getValue(), entry.getKey()));
                    Sidekick.logI("*** Wrapped VoltageSensor '%s'", entry.getKey());
                }
            } else {
                error(MinorError.VOLTAGE_SENSOR);
            }

            Sidekick.logI("*** Wrapped HardwareMap");
        }

        /// Wrap a single device object for archiving. `originalClass` is usually an
        /// interface like [DcMotorEx] derived from [HardwareDevice] but can actually be
        /// a made up class.
        DeviceInfo getDeviceInfo(Class<?> originalClass) {
            // See if a proxy class already exists in our cache, otherwise we'll have to create
            // it. We have a simple list rather than a HashMap because there are always so few
            // entries.
            for (DeviceInfo cache: deviceClassArray) {
                if (cache.originalClass == originalClass) {
                    return cache;
                }
            }

            // To save space in the descriptor table, determine the number of devices for this
            // type. Use the 'unsafe' iterable to avoid initializing all the devices:
            int deviceCount = Sidekick.isPC ? 10 : 0;
            for (HardwareDevice device : unsafeIterable()) {
                if (originalClass.isInstance(device)) {
                    deviceCount++;
                }
            }

            // Determine the bus type from the device class:
            BusType busType = BusType.OTHER;
            if (DcMotorEx.class.isAssignableFrom(originalClass)) { // MotorType doesn't work
                busType = BusType.MOTOR;
            } else if (hasAnnotation(originalClass, I2cDeviceType.class)) {
                busType = BusType.I2C;
            } else if (hasAnnotation(originalClass, AnalogSensorType.class)) {
                busType = BusType.ANALOG;
            } else if (hasAnnotation(originalClass, DigitalIoDeviceType.class)) {
                busType = BusType.DIGITAL;
            } else if (hasAnnotation(originalClass, ServoType.class)) {
                busType = BusType.SERVO;
            }

            // Create a cache entry::
            DeviceInfo deviceInfo = new DeviceInfo();
            deviceInfo.deviceId = deviceClassArray.size(); // Increment with every new ID
            deviceInfo.originalClass = originalClass;
            deviceInfo.busType = busType;
            deviceInfo.maxInstances = Math.max(deviceCount, 1); // Always at least one instance
            deviceInfo.isBulkReader = DcMotor.class.isAssignableFrom(originalClass) ||
                    AnalogInput.class.isAssignableFrom(originalClass) ||
                    DigitalChannel.class.isAssignableFrom(originalClass);
            deviceInfo.isPinpoint = GoBildaPinpointDriver.class.isAssignableFrom(originalClass);
            deviceInfo.isLimelight = Limelight3A.class.isAssignableFrom(originalClass);
            deviceClassArray.add(deviceInfo);
            return deviceInfo;
        }

        /// Create a proxy instance.
        @Nullable <T> T instantiateProxy(Class<?> proxyClass, Interceptor.Context context, T originalObject) {
            // Instantiate the proxy instance. We use Objenesis because it allows us to avoid
            // calling the constructor of the original class:
            @SuppressWarnings("unchecked")
            T proxyInstance = (T) Sidekick.objenesis.newInstance(proxyClass);

            // Now that we have the result, remember it and set the context and delegate fields:
            context.wrappedObject = proxyInstance;
            try {
                Field contextField = proxyClass.getDeclaredField(ProxyBuilder.CONTEXT_FIELD);
                contextField.setAccessible(true);
                contextField.set(proxyInstance, context);

                Field delegateField = proxyClass.getDeclaredField(ProxyBuilder.DELEGATE_FIELD);
                delegateField.setAccessible(true);
                delegateField.set(proxyInstance, originalObject);

                return proxyInstance;
            } catch (Exception ignored) {}
            return null;
        }

        /// Wrap a single device object for logging in response to a [HardwareMap#get] request
        /// (or one of the Get variants). We use Byte Buddy to programmatically create a derived
        /// class with all public methods serializing their parameters and forwarding to the
        /// original delegate object to do the actual work.
        <T> T wrapDevice(@NonNull T originalObject, @NonNull String deviceName) {
            // Check for some early outs:
            if (isStarted) {
                return originalObject; // ====> Don't wrap after Start is pressed; it takes too long!
            }
            if (originalObject instanceof SidekickProxy) {
                return originalObject; // ====> The object is already wrapped!
            }

            Class<?> originalClass = originalObject.getClass();
            deviceName = deviceName.trim();

            /// [ProxyBuilder#primeCache] is extremely slow (hundreds of milliseconds); call it
            /// outside of the lock (or have previously called it outside of the lock):
            sidekick.proxyBuilder.primeCache(originalClass);
            Class<?> proxyClass = sidekick.proxyBuilder.getCache(originalClass);
            if (proxyClass == null) {
                error(MinorError.PROXY_CLASS, "No proxy class for %s", originalClass.getSimpleName());
                return originalObject; // ====>
            }

            // Also pre-create proxy classes for any derivative devices associated with this
            // device. We wouldn't want the first call to limelight.getLatestResult() to hang
            // for hundreds of milliseconds while the robot is actually moving:
            for (DeviceChild derivative: DEVICE_CHILDREN) {
                if (derivative.deviceClass.isAssignableFrom(originalClass)) {
                    sidekick.proxyBuilder.primeCache(derivative.childClass);
                    /// Error is logged by caller of [ProxyBuilder#getCache]
                }
            }

            synchronized(sidekickLock) {
                DeviceInfo deviceInfo = getDeviceInfo(originalClass);

                // Check to see if this device name already exists for this class of device:
                int instanceId = deviceInfo.deviceNames.indexOf(deviceName);
                if (instanceId != -1) {
                    // The user previous called 'get' for this device; return the same instance:
                    return (T) deviceInfo.proxyInstances.get(instanceId); // ====>
                }

                // Determine the index to use as the instance identifier:
                instanceId = deviceInfo.deviceNames.size();
                if (instanceId >= deviceInfo.maxInstances) {
                    error(MinorError.PROXY_CLASS, "Too many instances of %s: %d", deviceName, instanceId);
                    return originalObject; // ====>
                }

                // Create the context for this particular device instance, then instantiate:
                Interceptor.Context context = new Interceptor.Context(this.capture, originalObject,
                        deviceInfo, 0, instanceId);
                T proxyInstance = instantiateProxy(proxyClass, context, originalObject);
                if (proxyInstance == null) {
                    error(MinorError.PROXY_CLASS);
                    return originalObject; // ====>
                }

                // Now that we're guaranteed success, commit the results:
                deviceInfo.deviceNames.add(deviceName);
                deviceInfo.proxyInstances.add((HardwareDevice) proxyInstance);

                // Do some device-specific special-case initialization:
                if (originalObject instanceof GoBildaPinpointDriver) {
                    // Record the Pinpoint's firmware version for optimization suggestions:
                    GoBildaPinpointDriver pinpoint = (GoBildaPinpointDriver) originalObject;
                    manifest.pinpointDeviceVersion = pinpoint.getDeviceVersion();
                }

                // If this is a webcam or built-in camera, remember to inform screen recording:
                if (originalObject instanceof CameraName) {
                    usingCamera = true;
                }

                // If this is the first Limelight seen, stash its IP address for video capture at Start:
                if ((deviceInfo.isLimelight) && (capture.limelightAddress == null)) {
                    InetAddress inetAddress = getField(originalObject, "inetAddress", InetAddress.class);
                    if (inetAddress != null) {
                        capture.limelightAddress = inetAddress.getHostAddress();
                    } else {
                        error(MinorError.LIMELIGHT_VIDEO_CAPTURE, "Couldn't get Limelight IP address.");
                    }
                }
                return proxyInstance;
            }
        }

        /// Wrap a select class returned from a wrapped [HardwareDevice] object.
        <T> T wrapChild(@NonNull T originalObject, DeviceInfo deviceInfo, int instanceId, int childId) {
            if (!(originalObject instanceof SidekickProxy)) {
                Class<?> proxyClass = sidekick.proxyBuilder.getCache(originalObject.getClass());
                if (proxyClass != null) {
                    // Create the context for this particular device instance and instantiate:
                    Interceptor.Context context = new Interceptor.Context(this.capture,
                            originalObject, deviceInfo, childId, instanceId);
                    T proxyInstance = instantiateProxy(proxyClass, context, originalObject);
                    if (proxyInstance != null) {
                        return proxyInstance; // ====> Success!
                    }
                }
            }

            // Failure case:
            error(MinorError.PROXY_CLASS);
            return originalObject;
        }

        /// Helper for [#hasAnnotation]
        private Class<? extends Annotation> getRepeatableContainer(Class<? extends Annotation> annotation) {
            Repeatable rep = annotation.getAnnotation(Repeatable.class);
            return rep == null ? null : rep.value();
        }

        /// Determine if a class or its superclasses or any of its interfaces has an annotation.
        boolean hasAnnotation(Class<?> klass, Class<? extends Annotation> annotation) {
            Class<?> current = klass;
            while (current != null) {
                // Check direct annotation:
                if (current.isAnnotationPresent(annotation)) {
                    return true;
                }

                // Check repeatable container:
                Class<? extends Annotation> container = getRepeatableContainer(annotation);
                if (container != null && current.isAnnotationPresent(container)) {
                    return true;
                }

                // Check interfaces:
                for (Class<?> interfaceClass: current.getInterfaces()) {
                    if (interfaceClass.isAnnotationPresent(annotation)) {
                        return true;
                    }
                    if (container != null && interfaceClass.isAnnotationPresent(container)) {
                        return true;
                    }
                }
                current = current.getSuperclass();
            }
            return false;
        }

        /// [HardwareMap#get] and all variants returns a wrapped Sidekick instrumented proxy.
        @Override public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
            long nanoStartTime = nanoTime();
            T device = super.get(classOrInterface, deviceName); // Result is guaranteed non-null
            recordHardwareMapGet("get", nanoStartTime, classOrInterface, deviceName);
            return wrapDevice(device, deviceName);
        }

        @Override public HardwareDevice get(String deviceName) {
            long nanoStartTime = nanoTime();
            HardwareDevice device = super.get(deviceName); // Result is guaranteed non-null
            recordHardwareMapGet("get", nanoStartTime, device.getClass(), deviceName);
            return wrapDevice(device, deviceName);
        }

        @Override public @Nullable <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
            long nanoStartTime = nanoTime();
            T device = super.tryGet(classOrInterface, deviceName);
            if (device == null) {
                return null;
            }
            recordHardwareMapGet("tryGet", nanoStartTime, classOrInterface, deviceName);
            return wrapDevice(device, deviceName);
        }

        @Override public @Nullable <T> T get(Class<? extends T> classOrInterface, SerialNumber serialNumber) {
            long nanoStartTime = nanoTime();
            T device = super.get(classOrInterface, serialNumber);
            if (device == null) {
                return null;
            }
            String deviceName = SerialNumber.getDeviceDisplayName(serialNumber);
            recordHardwareMapGet("get", nanoStartTime, classOrInterface, deviceName);
            return wrapDevice(device, deviceName);
        }

        @Override public <T> List<T> getAll(Class<? extends T> classOrInterface) {
            long nanoStartTime = nanoTime();
            List<T> originals = super.getAll(classOrInterface);
            List<T> result = new LinkedList<>();
            for (T device: originals) {
                /// When we call wrapDevice(), we have to supply the device's configuration name.
                /// [HardwareMap#getNamesOf] says there can (rarely) be multiple names for a device.
                /// I have no idea why that would be, so we'll just use the first one, if any:
                Set<String> names = super.getNamesOf((HardwareDevice) device);
                String name = !names.isEmpty() ? names.iterator().next() : "???";

                // We cheesily return a record for each device found:
                recordHardwareMapGet("getAll", nanoStartTime, classOrInterface, name);
                result.add(wrapDevice(device, name));

                // The first device record gets the actual duration; subsequent device records
                // returned by the call appear with very little consumed time:
                nanoStartTime = nanoTime();
            }
            return result;
        }

        @Override public @NonNull Set<String> getNamesOf(HardwareDevice device) {
            return super.getNamesOf(Sk.unwrap(device));
        }
    }
}
