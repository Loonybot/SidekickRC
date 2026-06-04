#include <jni.h>
#include <stdint.h>

JNIEXPORT jlong JNICALL
Java_com_loonybot_sidekick_FastClock_readCounter(JNIEnv* env, jclass clazz) {
    uint64_t cnt;
    asm volatile("mrs %0, cntvct_el0" : "=r"(cnt));
    return (jlong)cnt;
}

JNIEXPORT jlong JNICALL
Java_com_loonybot_sidekick_FastClock_counterFrequency(JNIEnv* env, jclass clazz) {
    uint64_t freq;
    asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return (jlong)freq;
}
