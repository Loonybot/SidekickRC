package com.loonybot.sidekick;

// TODO: Delete this!

public final class FastClock {
    private static final boolean nativeAvailable;

    static {
        boolean ok = false;
        try {
            System.loadLibrary("sidekicktimer");
            ok = true;
        } catch (Throwable t) {
            ok = false;
        }
        nativeAvailable = ok;
    }

    private static native long readCounter();
    private static native long counterFrequency();

    public static final long freq = nativeAvailable ? counterFrequency() : 1;

    public static long now() {
        if (!nativeAvailable) {
            return System.nanoTime();
        }
        long ticks = readCounter();
//        Log.d("Sidekick", String.format("Ticks: %,d", ticks));
        return (ticks * 1_000_000_000L) / freq;
    }
}
