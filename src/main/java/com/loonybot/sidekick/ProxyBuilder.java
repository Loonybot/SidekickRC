/// Builder for the Byte Buddy proxies that wrap the SDK HardwareDevice objects.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import android.content.Context;

import androidx.annotation.NonNull;

import net.bytebuddy.ByteBuddy;
import net.bytebuddy.NamingStrategy;
import net.bytebuddy.android.AndroidClassLoadingStrategy;
import net.bytebuddy.description.modifier.Visibility;
import net.bytebuddy.description.type.TypeDescription;
import net.bytebuddy.dynamic.DynamicType;
import net.bytebuddy.dynamic.loading.ClassLoadingStrategy;
import net.bytebuddy.implementation.FieldAccessor;
import net.bytebuddy.implementation.MethodDelegation;
import net.bytebuddy.matcher.ElementMatchers;

import java.util.concurrent.ConcurrentHashMap;

/// Byte Buddy proxy builder.
public class ProxyBuilder {
    final static String CONTEXT_FIELD = "_sidekickContext"; // Private context
    final static String DELEGATE_FIELD = "_sidekickDelegate"; // Used for unwrap()

    final ClassLoadingStrategy<ClassLoader> classLoadingStrategy; // ByteBuddy loading strategy
    ConcurrentHashMap<Class<?>, Class<?>> proxyCache = new ConcurrentHashMap<>(); // Cache of proxy classes for each type

    /// Create the appropriate class loading strategy for Byte Buddy to use.
    ProxyBuilder(Context context) {
        if (Sidekick.isPC) {
            classLoadingStrategy = ClassLoadingStrategy.Default.INJECTION;
        } else {
            classLoadingStrategy = new AndroidClassLoadingStrategy.Injecting(context.getDir(
                    "generated",
                    Context.MODE_PRIVATE));
        }
    }

    /// Create a proxy class via Byte Buddy that wraps the specified class or interface. Returns
    /// the original in the event of failure. Never returns null - in the event of failure, it
    /// returns the original.
    private Class<?> createProxyClass(@NonNull Class<?> originalClass) {
        long startTime = System.nanoTime();
        try (DynamicType.Unloaded<?> unloaded = new ByteBuddy()
                .with(new NamingStrategy.AbstractBase() {
                    @Override @NonNull
                    protected String name(@NonNull TypeDescription superClass) {
                        return superClass.getName() + "$SidekickProxy";
                    }
                })
                .subclass(originalClass)
                .implement(SidekickProxy.class)
                .defineField(CONTEXT_FIELD, Interceptor.Context.class, Visibility.PRIVATE)
                .defineField(DELEGATE_FIELD, originalClass, Visibility.PRIVATE)
                .defineMethod("unwrap", originalClass, Visibility.PUBLIC)
                .intercept(FieldAccessor.ofField(DELEGATE_FIELD))
                .method(
                        ElementMatchers.isPublic()
                                .and(ElementMatchers.not(ElementMatchers.isConstructor()))
                                .and(ElementMatchers.not(ElementMatchers.isDeclaredBy(Object.class)))
//                                .and(ElementMatchers.not(ElementMatchers.isDeclaredBy(HardwareDevice.class)))
                                .and(ElementMatchers.not(ElementMatchers.named("unwrap")))
                )
                .intercept(MethodDelegation.to(new Interceptor(originalClass)))
                .make()) {
            return unloaded.load(getClass().getClassLoader(), classLoadingStrategy)
                    .getLoaded();
        } catch (Exception e) {
            return originalClass;
        } finally {
            double durationMs = (System.nanoTime() - startTime) / 1e6;
            Sidekick.logI("Wrapped %s in %.0f ms", originalClass.getSimpleName(), durationMs);
        }
    }

    /// Build and cache a proxy for the specified class. This will take 100s of milliseconds
    /// so should be done without holding the Sidekick lock. It's okay to call this while
    /// holding the Sidekick lock ***if*** the proxy has previously been primed.
    void primeCache(Class<?> original) {
        // Do NOT hold the Sidekick lock while calling because this function can take >100ms!
        assert(!Thread.holdsLock(Sidekick.sidekickLock));

        /// [#createProxyClass] never returns null so we're safe to use [ConcurrentHashMap] here:
        proxyCache.computeIfAbsent(original, this::createProxyClass); // Create the proxy if necessary
    }

    /// Get the proxy from the cache. Does not create a new proxy so [#primeCache(Class)] must have
    /// already been called. Returns null if it wasn't found or if the original object couldn't
    /// be wrapped.
    Class<?> getCache(Class<?> original) {
        Class<?> proxy = proxyCache.get(original);

        /// We can't store 'null' in a [#proxyCache] so failure is marked by an unwrapped object:
        if ((proxy == null) || !(SidekickProxy.class.isAssignableFrom(proxy))) {
            return null;
        }
        return proxy;
    }
}
