/// The common proxy interceptor code responsible for wrapping and recording every SDK call.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import static com.loonybot.sidekick.Capture.TICK_MASK;
import static com.loonybot.sidekick.Capture.TICK_SHIFT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import net.bytebuddy.implementation.bind.annotation.AllArguments;
import net.bytebuddy.implementation.bind.annotation.FieldValue;
import net.bytebuddy.implementation.bind.annotation.Origin;
import net.bytebuddy.implementation.bind.annotation.RuntimeType;

import java.lang.reflect.Method;

/// This class intercepts all calls to a Hardware Device class or interface via Byte Buddy
/// method delegation. It times and calls the original method and then serializes the arguments
/// and result.
@SuppressWarnings("unchecked")
public class Interceptor {
    Class<?> originalClass; // This interceptor applies to this class type

    /// Create the interceptor for the specified class.
    Interceptor(Class<?> originalClass) {
        this.originalClass = originalClass;
    }

    /// Flags indicating special handling of a method call.
    public static class Flags {
        static final int WRAP_CHILD = 0x1; // Wrap the return class as a child
        static final int LYNX_BULK_READ = 0x2; // Lynx bulk read candidate
        static final int LIMELIGHT_STATUS_UPDATE = 0x4; // Limelight3A status update
        static final int PINPOINT_BULK_READ = 0x8; // Pinpoint bulk read candidate
        static final int PINPOINT_STATUS_UPDATE = 0x10; // Pinpoint status update
        static final int PINPOINT_FLAGS = PINPOINT_BULK_READ | PINPOINT_STATUS_UPDATE;
    }

    /// Per-hardware-device context for the interceptor.
    public static class Context {
        final Capture capture; // Associated Sidekick capture
        final Object originalObject; // Delegate device object
        final DeviceInfo deviceInfo; // Associated device class info
        final int childId; // ID of the child class with this context; 0 if not a child
        final int instanceId; // Instance ID for this device, added to recordId at capture

        Object wrappedObject; // Proxy device object, set later

        Context(Capture capture, Object originalObject, DeviceInfo deviceInfo, int childId, int instanceId) {
            this.capture = capture;
            this.originalObject = originalObject;
            this.deviceInfo = deviceInfo;
            this.childId = childId;
            this.instanceId = instanceId;
        }
    }

    /// [#intercept] uses a HashMap of the Method object to cache our serialization data; we
    /// want that data to be unique based on the top-level class name, method name, and parameter
    /// types. The Method object alone guarantees non-collisions based on the defining class name,
    /// method name, and parameter types, as per the
    /// [docs](https://docs.oracle.com/javase/8/docs/api/java/lang/reflect/Method.html#equals-java.lang.Object-).
    /// We create this key to supplement the original Method object to incorporate the top-level
    /// class; if we didn't do this, methods defined by a common base class (such as HardwareDevice)
    /// will have collisions and get misattributed.
    class MethodKey {
        public Method method;
        public Class<?> topLevelClass;

        MethodKey(Method method) {
            this.method = method;
            this.topLevelClass = originalClass;
        }

        @Override public boolean equals(Object o) {
            if (this == o)
                return true;
            if (!(o instanceof MethodKey)) // Yes, this check is necessary
                return false;
            MethodKey other = (MethodKey) o;
            return this.method.equals(other.method) && this.topLevelClass.equals(other.topLevelClass);
        }

        @Override public int hashCode() {
            // This expression could also be "Objects.hash(method, topLevelClass)":
            return this.method.hashCode() ^ this.topLevelClass.hashCode();
        }
    }

    /// Every user call to a HardwareDevice or child method is intercepted by this routine.
    @RuntimeType public Object intercept(
            @Origin Method method,
            @AllArguments Object[] args,
            @FieldValue(ProxyBuilder.CONTEXT_FIELD) Context context
    ) throws Exception {
        Capture.RecordDescriptor descriptor;

        // Call the original method on the original object. We can't use @SuperCall because
        // the proxy instance has uninitialized fields due to Objenesis:
        long nanoStartTime = System.nanoTime();
        Object result = method.invoke(context.originalObject, args);
        long nanoDuration = System.nanoTime() - nanoStartTime;

        // Now store the arguments and the results:
        Capture capture = context.capture;
        assert(!Thread.holdsLock(capture.sidekickLock));
        synchronized (capture.sidekickLock) {
            if (Thread.currentThread().getId() != capture.currentThreadId) {
                if (!capture.threadSwitchAndBookkeeping())
                    return result; // ====>
            }

            // An alternative to using MethodKey would be to maintain method HashMaps per-class in
            // the Interceptor instance; that would require the map to be reset on every new capture.
            MethodKey methodKey = new MethodKey(method);
            descriptor = capture.methodDescriptors.get(methodKey);
            if (descriptor == null) {
                descriptor = capture.new RecordDescriptor(context, methodKey);
            }

            // Now record the method call:
            if ((capture.chunkRemaining -= (descriptor.recordLength + 8)) < 0) {
                capture.nextChunk();
            }
            long recordId = descriptor.recordId + context.instanceId;
            long startTicks = (nanoStartTime >> TICK_SHIFT) & TICK_MASK;
            long durationTicks = (nanoDuration >> TICK_SHIFT) & TICK_MASK;
            capture.chunk.putLong((recordId << 48) | (startTicks << 24) | (durationTicks));
            for (int i = 0; i < descriptor.paramPutters.length; i++) {
                descriptor.paramPutters[i].accept(args[i]);
            }
            descriptor.resultPutter.accept(result);
            capture.assertChunk();
        }

        // Now that we're no longer holding the lock, handle special cases that can take a while:
        if (descriptor.interceptorFlags != 0) {
            // Wrap the return if it's a child class:
            if (((descriptor.interceptorFlags & Flags.WRAP_CHILD) != 0) && (result != null)) {
                result = capture.hardwareMap.wrapChild(result, context.deviceInfo, context.instanceId, descriptor.wrapId);
            }
            // If this method is a Lynx module bulk reader candidate, then record the bulk data:
            if ((descriptor.interceptorFlags & Flags.LYNX_BULK_READ) != 0) {
                capture.recordLynxBulkRead(capture, descriptor.lynxInfo);
            }
            /// Record the Limelight's status when [Limelight3A#getLatestResult()] is called:
            if ((descriptor.interceptorFlags & Flags.LIMELIGHT_STATUS_UPDATE) != 0) {
                capture.markBonus(true);
                ((Limelight3A) context.wrappedObject).getStatus(); // Effectively recursive
                capture.markBonus(false);
            }
            /// Record additional data when [GoBildaPinpointDriver#update()] is called:
            if ((descriptor.interceptorFlags & Flags.PINPOINT_FLAGS) != 0) {
                capture.recordPinpointUpdate((GoBildaPinpointDriver) context.wrappedObject, descriptor.interceptorFlags);
            }
        }
        return result;
    }
}