/// The common proxy interceptor code responsible for wrapping and recording every SDK call.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import static com.loonybot.sidekick.Capture.TICK_MASK;
import static com.loonybot.sidekick.Capture.TICK_SHIFT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import net.bytebuddy.implementation.bind.annotation.AllArguments;
import net.bytebuddy.implementation.bind.annotation.FieldValue;
import net.bytebuddy.implementation.bind.annotation.Origin;
import net.bytebuddy.implementation.bind.annotation.RuntimeType;

import java.lang.reflect.Method;

/// This class intercepts all calls to a Hardware Device class or interface via Byte Buddy
/// method delegation. It times and calls the original method and then serializes the arguments
/// and result.
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
        static final int PINPOINT_BULK_READ = 0x4; // Pinpoint bulk read candidate
        static final int PINPOINT_STATUS_UPDATE = 0x08; // Pinpoint status update
        static final int PINPOINT_FLAGS = PINPOINT_BULK_READ | PINPOINT_STATUS_UPDATE;
    }

    /// Per-hardware-instance context for the interceptor.
    public static class Context {
        final Capture capture; // Associated Sidekick capture
        final Object originalObject; // Delegate device object
        final DeviceClassInfo classInfo; // Associated device class info
        final int childId; // ID of the child class with this context; 0 if not a child
        final int instanceId; // ID for this device instance, added to recordId at capture

        Object wrappedObject; // Proxy device object, set later

        Context(Capture capture, Object originalObject, DeviceClassInfo classInfo, int childId, int instanceId) {
            this.capture = capture;
            this.originalObject = originalObject;
            this.classInfo = classInfo;
            this.childId = childId;
            this.instanceId = instanceId;
        }
    }

    /// Every user call to a HardwareDevice or child method is intercepted by this routine. For
    /// fast lookups of the DynamicDescriptor, the Method object guarantees uniqueness (no
    /// collisions) based on the defining class name, method name, and parameter types, as per the
    /// [docs](https://docs.oracle.com/javase/8/docs/api/java/lang/reflect/Method.html#equals-java.lang.Object-).
    @SuppressWarnings("unused") // ByteBuddy generates references to this method
    @RuntimeType public Object intercept(
            @Origin Method method,
            @AllArguments Object[] args,
            @FieldValue(ProxyBuilder.CONTEXT_FIELD) Context context // Context for this instance
    ) throws Exception {
        Capture.DynamicDescriptor descriptor;

        // Call the original method on the original object. We can't use @SuperCall because
        // the proxy instance has uninitialized fields due to Objenesis:
        long nanoStartTime = System.nanoTime();
        Object result = method.invoke(context.originalObject, args);

        // Now store the arguments and the results:
        Capture capture = context.capture;
        synchronized (capture.sidekickLock) {
            long nanoEndTime = System.nanoTime();

            if (Thread.currentThread().getId() != capture.currentThreadId) {
                if (!capture.threadSwitchAndBookkeeping())
                    return result; // ====>
            }

            // Find this method's descriptor in the hash map (creating it if necessary):
            descriptor = context.classInfo.descriptorMap.computeIfAbsent(method,
                m -> capture.new DynamicDescriptor(context, m));

            // Now record the method call:
            if ((capture.chunkRemaining -= (descriptor.recordLength + 8)) < 0) {
                capture.nextChunk();
            }
            long recordId = descriptor.recordId + context.instanceId;
            long endTicks = (nanoEndTime >> TICK_SHIFT) & TICK_MASK;
            long durationTicks = ((nanoEndTime - nanoStartTime) >> TICK_SHIFT);
            if (durationTicks >= TICK_MASK) {
                durationTicks = TICK_MASK; // Saturate duration, effectively capping to 4 seconds
            }
            capture.chunk.putLong((recordId << 48) | (endTicks << 24) | (durationTicks));
            for (int i = 0; i < descriptor.paramPutters.size(); i++) {
                descriptor.paramPutters.get(i).accept(args[i]);
            }
            descriptor.resultPutter.accept(result);
            capture.assertChunk();
        }

        // Now that we're no longer holding the lock, handle special cases that can take a while:
        if (descriptor.interceptorFlags != 0) {
            // Wrap the return if it's a child class:
            if (((descriptor.interceptorFlags & Flags.WRAP_CHILD) != 0) && (result != null)) {
                result = capture.hardwareMap.wrapChild(result, context.classInfo, context.instanceId, descriptor.wrapId);
            }
            // If this method is a Lynx module bulk reader candidate, then record the bulk data:
            if ((descriptor.interceptorFlags & Flags.LYNX_BULK_READ) != 0) {
                capture.recordLynxBulkRead(capture, descriptor.lynxState);
            }
            /// Record additional data when [GoBildaPinpointDriver#update()] is called:
            if ((descriptor.interceptorFlags & Flags.PINPOINT_FLAGS) != 0) {
                capture.recordPinpointUpdate((GoBildaPinpointDriver) context.wrappedObject,
                        context.instanceId, descriptor.interceptorFlags);
            }
        }
        return result;
    }
}