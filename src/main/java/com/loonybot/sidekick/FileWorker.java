/// File writer worker thread logic.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import static com.loonybot.sidekick.Sidekick.SUBDIRECTORY;
import static java.lang.System.nanoTime;

import android.system.ErrnoException;
import android.system.Os;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.File;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/// All file writing is handled by this background thread to avoid blocking the primary opMode
/// thread on file I/O. A message queue is employed for the primary thread to enqueue file
/// requests. Android has nice thread message queue support which we don't use so that this
/// code can be tested on the PC.
@SuppressWarnings("NonAtomicOperationOnVolatileField")
class FileWorker {
    /// The main Sidekick thread communicates to our thread with the following message template.
    static abstract class Message {}

    /// Message to open the capture file. It will use a temporary name that incorporates the current
    /// date and time.
    static class OpenMessage extends Message {
        public final String dateAndTime;
        public OpenMessage(String dateAndTime) {
            this.dateAndTime = dateAndTime;
        }
    }

    /// Message to write a buffer to the capture file.
    static class WriteMessage extends Message {
        final ByteBuffer buffer;
        public WriteMessage(ByteBuffer buffer) {
            this.buffer = buffer;
        }
    }

    /// Message to write the file's header record and close the capture file.
    static class CloseMessage extends Message {
        int majorErrors;
        int minorErrors;
        Section[] sections;
        Class<? extends OpMode> opModeClass;
        String dateAndTime;
        long startUnixTime;
        long startNanoTime;
        long endNanoTime;
        int lastRunNumber;
        CountDownLatch limelightDoneLatch;
        CountDownLatch fileDoneLatch;
        public CloseMessage(int majorErrors, int minorErrors, Section[] sections, Class<? extends OpMode> opModeClass, String dateAndTime, long startUnixTime, long startNanoTime, long endNanoTime, int lastRunNumber, CountDownLatch limelightDoneLatch, CountDownLatch fileDoneLatch) {
            this.majorErrors = majorErrors;
            this.minorErrors = minorErrors;
            this.sections = sections;
            this.opModeClass = opModeClass;
            this.dateAndTime = dateAndTime;
            this.startUnixTime = startUnixTime;
            this.startNanoTime = startNanoTime;
            this.endNanoTime = endNanoTime;
            this.lastRunNumber = lastRunNumber;
            this.limelightDoneLatch = limelightDoneLatch;
            this.fileDoneLatch = fileDoneLatch;
        }
    }

    static final double MIN_MATCH_TELOP_DURATION = 115; // Min run seconds to be considered a match TeleOp
    static final double MATCH_TIME_EPSILON = 2; // Fudge for considering interval between OpModes, seconds
    static final int CHUNK_SIZE = 500; // @@@ // Size of our working ByteBuffers, in bytes TODO: fix
    static final int PRE_ALLOCATED_CHUNK_COUNT = 4; // Number of pre-allocated chunk ByteBuffers
    static final int MAX_CAPTURE_SIZE = 100*1024*1024; // Stop capturing at 100 MB

    // We pre-prime the wrappers for the following classes which are pretty much guaranteed to be
    // used by every TeleOp (the LynxVoltageSensor is used by the system, the Lynx Module to
    // set the bulk caching mode, and real-time counters may sample AnalogInput):
    static final Class<?>[] PRIME_CLASSES = { LynxModule.class, DcMotorEx.class, AnalogInput.class };

    static int retentionDays = 7; // Default retention period, in days
    ProxyBuilder proxyBuilder; // Byte Buddy proxy builder
    LinkedBlockingQueue<Message> messageQueue = new LinkedBlockingQueue<>(); // Message queue
    ConcurrentLinkedQueue<ByteBuffer> freeBufferPool = new ConcurrentLinkedQueue<>(); // List of available buffers
    volatile static int bytesQueued; // Number of bytes queued for writing, read by SamplerWorker
    volatile static int bytesWritten; // Number of bytes actually written, read by SamplerWorker
    boolean fileError; // True if a file error occurred while writing
    FileOutputStream stream; // Output stream
    FileChannel channel; // Output channel (note that AsynchronousFileChannel is not a win)
    File file; // Output file
    volatile boolean running = true; // True if the worker thread is running
    String previousFilename = ""; // File name of the preceding capture, if any
    double previousStartSeconds; // Previous start time, in seconds
    boolean previousWasAuto; // True if previous capture (if any) was an Auto

    /// The file worker thread's entry point.
    private void loop() {
        Thread.currentThread().setName(Sidekick.FILE_WORKER_THREAD_NAME);
        Sidekick.instance.registerThreadStart(Sidekick.FILE_WORKER_THREAD_NAME);

        // We take the opportunity now to do some pre-processing in parallel with the primary
        // thread. The first proxy call to Byte Buddy is stupendously slow (presumably because
        // Byte Buddy has a ton of code to JIT compile), whereas subsequent calls are merely
        // extremely slow (still many hundreds of milliseconds). Reduce the user's eventual
        // startup time by pre-creating some proxy classes on this thread before INIT is even
        // pressed.
        for (Class<?> klass: PRIME_CLASSES) {
            proxyBuilder.primeCache(klass);
        }

        // Signal that the priming is done:
        Sidekick.primingDoneSignal.countDown();

        // Now run our main file worker loop:
        try {
            while (running) {
                process(messageQueue.take());
            }
        } catch (InterruptedException ignored) {}
    }

    /// Process the next request message in the queue.
    private void process(Message message) {
        try {
            if (message instanceof OpenMessage) {
                open(((OpenMessage) message).dateAndTime);
            } else if (message instanceof WriteMessage) {
                write(((WriteMessage) message).buffer);
            } else if (message instanceof CloseMessage) {
                close((CloseMessage) message);
            }
        } catch (IOException e) {
            Sidekick.logE("File I/O error: %s", e.getMessage());
            fileError = true;
        }
    }

    /// Open the capture file.
    private void open(String dateAndTime) throws IOException {
        // Close any existing stream and reset the error flag:
        closeStream();

        // Give the capture a temporary name until the capture is complete, of the form of
        // "2024-11-25 [17.37pm] Sidekick.temporary":
        String path = SUBDIRECTORY + "/" + dateAndTime + ".temporary";
        file = new File(path);
        stream = new FileOutputStream(file);
        channel = stream.getChannel();
        fileError = false;
        bytesWritten = 0;
    }

    /// Append a completed buffer to the capture file.
    private void write(ByteBuffer chunk) throws IOException {
        int byteCount = chunk.position();
        chunk.flip(); // Flip from write-mode to read-mode
        do {
            channel.write(chunk);
        } while (chunk.hasRemaining());
        chunk.clear(); // Clear and switch back to write-mode
        freeBufferPool.offer(chunk);
        bytesWritten += byteCount;
    }

    /// Close the capture stream.
    private void closeStream() {
        try {
            if (channel != null) {
                syncBeforeClose(stream);
                stream.close();
                syncAfterClose(file);
            }
        } catch (IOException ignored) {
            Sidekick.logE("Failed to close file %s", file.getAbsolutePath());
        }

        stream = null;
        channel = null;
        file = null;

        // Free excess buffers from the pool:
        while (freeBufferPool.size() > PRE_ALLOCATED_CHUNK_COUNT) {
            freeBufferPool.poll();
        }
    }

    /// Write the capture's header, close the file, and rename it.
    private void close(CloseMessage message) throws IOException {
        // Write the file header to the beginning of the file and then close the stream. First
        // add ny file I/O errors to the message:
        message.majorErrors |= (fileError) ? (1 << MajorError.FILE_WRITE_ERROR.ordinal()) : 0;
        ByteBuffer headerBuffer = Header.create(message.majorErrors, message.minorErrors,
                message.startUnixTime, message.sections);
        do {
            channel.write(headerBuffer, 0);
        } while (headerBuffer.hasRemaining());

        float startSeconds = message.startNanoTime / 1e9f;
        double runSeconds = (message.endNanoTime - message.startNanoTime) / 1e9f;

        File captureFile = file; /// Grab a reference before [#closeStream()] nulls it
        closeStream();

        // Apply a heuristic to determine if this is the end of a match. The first test is whether
        // this is a TeleOp, and it lasted more than two minutes, and the previous capture was an
        // Autonomous:
        boolean isTeleOp = message.opModeClass.isAnnotationPresent(TeleOp.class);
        boolean isMatchTeleOp = false;
        if ((isTeleOp) && (previousWasAuto) && (runSeconds > MIN_MATCH_TELOP_DURATION)) {
            // For the next test, decide if the TeleOp started about 38 seconds after the
            // Autonomous:
            double intervalBetweenStarts = startSeconds - previousStartSeconds;
            if ((intervalBetweenStarts >= 38 - MATCH_TIME_EPSILON) &&
                    (intervalBetweenStarts <= 38 + MATCH_TIME_EPSILON)) {
                // Yes, this capture is for a match TeleOp and the preceding capture was
                // for a match Autonomous:
                isMatchTeleOp = true;

                // Rename the preceding capture to add "Match":
                if (!previousFilename.contains("(Match)")) {
                    File previousFile = new File(SUBDIRECTORY, previousFilename);
                    String newName = previousFilename.replace(".sidekick", " (Match).sidekick");
                    if (!previousFile.renameTo(new File(SUBDIRECTORY, newName))) {
                        Sidekick.logE("Couldn't rename previous capture");
                    }
                }
            }
        }

        String descriptor = message.opModeClass.getSimpleName();
        if (isMatchTeleOp)
            descriptor += " (Match)";

        // Create the final filename in the form "2024-11-25 [17.37pm] #1 [176s] TeleOp.sidekick":
        String fileName = message.dateAndTime + String.format(Locale.US, " #%d [%.0fs] %s.sidekick",
                message.lastRunNumber, runSeconds, descriptor);

        // Before potentially creating a new epoch capture, delete all old ones:
        deleteEpochCaptures();

        // Once the capture file is renamed in the step below, the capture will be immediately
        // available for download by the Sidekick app, including all companion files (namely
        // the Limelight capture file). We need to ensure here that all companion files are
        // also complete - except for the screen recording file that is synchronized on download.
        if (message.limelightDoneLatch != null) {
            try {
                message.limelightDoneLatch.await(5, TimeUnit.SECONDS);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }

        // Our final step is to rename the temporary capture to its final name. Once this happens,
        // the capture can be immediately downloaded by the app.
        if (!captureFile.renameTo(new File(SUBDIRECTORY, fileName))) {
            Sidekick.logE("Couldn't rename saved capture when closing.");
            fileError = true;
        }

        // Signal that the file is completely written:
        message.fileDoneLatch.countDown();

        if (!fileError) {
            // We successfully saved the capture!
            previousWasAuto = !isTeleOp;
            previousFilename = fileName;
            previousStartSeconds = startSeconds;

            double closeMilliseconds = (nanoTime() - message.endNanoTime) * 1e-6;
            Sidekick.logI("The capture was saved in %.1f ms with a size of %,d bytes!",
                    closeMilliseconds, FileWorker.bytesQueued);
        } else {
            // This capture isn't usable so clean up:
            previousWasAuto = false;
            previousFilename = "";
            previousStartSeconds = 0;
        }
    }

    ///---------------------------------------------------------------------------------------------
    /// Everything below is callable from other threads.

    /// Get the date and time from the final capture name.
    /// Converts "2024-11-25 [17.37pm] #1 [176s] TeleOp.sidekick" to "2024-11-25 [17.37pm]".
    /// For "Incomplete" captures with no hash, it returns the complete name.
    static String getDateAndTimeFromCaptureName(String captureName) {
        int hashIndex = captureName.indexOf(" #");
        return (hashIndex != -1) ? captureName.substring(0, captureName.indexOf(" #")) : captureName;
    }

    /// Start the worker thread and pre-allocate some buffers.
    FileWorker(ProxyBuilder proxyBuilder) {
        this.proxyBuilder = proxyBuilder;
        new Thread(this::loop).start();
        for (int i = 0; i < PRE_ALLOCATED_CHUNK_COUNT; i++) {
            freeBufferPool.offer(ByteBuffer.allocateDirect(CHUNK_SIZE));
        }
    }

    /// Flush before closing to guard against data loss when the robot is turned off. This
    /// ensures the file's contents are saved to storage.
    static void syncBeforeClose(FileOutputStream stream) {
        try {
            // Ensure the file inode + initial data are durable:
            stream.getFD().sync();
            // Native-level sync (FTC Android sometimes needs this):
            Os.fsync(stream.getFD());
        } catch (IOException | ErrnoException e) {
            Sidekick.logE("File durability sync failed: %s", e.getMessage());
        }
    }

    /// Flush after closing for further durability. This ensures the *directory* entry is flushed.
    static void syncAfterClose(File file) {
        File parent = Objects.requireNonNull(file.getParentFile());
        FileDescriptor dirFd = null;
        try {
            // Ensure the directory entry is durable:
            final int O_DIRECTORY = 524288; // Linux constant, 02000000 octal
            dirFd = Os.open(parent.getAbsolutePath(), O_DIRECTORY, 0);
            Os.fsync(dirFd);
        } catch (ErrnoException e) {
            Sidekick.logE("Directory durability sync failed: %s", e.getMessage());
        } finally {
            if ((dirFd != null) && (dirFd.valid())) {
                try {
                    Os.close(dirFd);
                } catch (ErrnoException ignored) {}
            }
        }
    }

    /// Delete all captures dated 1969 or 1970. It's impossible to determine the true age of
    /// captures that are dated 1969 or 1970 as those are made when the Control Hub doesn't
    /// have a Driver Hub connection and an opMode is run by means such as FTC Dashboard. The
    /// Control Hub doesn't know the current time and date and so uses the start of the Unix
    /// epoch in these cases, hence the 1969 and 1970. We employ a simple policy to avoid the
    /// many ambiguities and complications that result: any existing capture dated 1969 or 1970
    /// is deleted from the Control Hub as soon as a new capture is made. You're free to download
    /// and save every capture on your PC, though, of course.
    static void deleteEpochCaptures() {
        File directory = new File(Sidekick.SUBDIRECTORY);
        FilenameFilter filter = (dir, name) -> name.endsWith(".sidekick") &&
                (name.startsWith("1969-") || name.startsWith("1970-"));
        File[] files = directory.listFiles(filter);
        if (files != null) {
            for (File file: files) {
                file.delete();
            }
        }
    }

    /// Submit a message to the worker thread.
    void submit(Message message) {
        if (message instanceof OpenMessage) {
            bytesQueued = 0;
        }
        if (message instanceof WriteMessage) {
            bytesQueued += ((WriteMessage) message).buffer.position();
        }
        messageQueue.offer(message);
    }

    /// From the caller's perspective, write and flip a ByteBuffer chunk so that it can be
    /// immediately reused. Under the covers, we queue the buffer for write by the
    /// asynchronous file worker thread. We return a previously queued chunk that was
    /// successfully written and now free, if any; otherwise, we allocate a new buffer and
    /// return that. We don't worry about backpressure causing too much ByteBuffer memory
    /// to be allocated, as we control for that with the excessive-capture-size check
    /// where [FileWorker#MAX_CAPTURE_SIZE] is the (approximate) upper-limit for amount of
    /// memory allocated.
    ByteBuffer writeAndFlipChunk(ByteBuffer chunk) {
        submit(new WriteMessage(chunk));
        return getChunk();
    }

    /// Get a free ByteBuffer chunk.
    ByteBuffer getChunk() {
        ByteBuffer chunk = freeBufferPool.poll();
        if (chunk == null) {
            chunk = ByteBuffer.allocateDirect(CHUNK_SIZE);
        }
        return chunk;
    }

    /// Return the total number of bytes queued.
    int getBytesQueued() {
        return bytesQueued;
    }

    /// Return true if the capture has grown excessively large and should be ended.
    boolean isExcessive() {
        return bytesQueued > MAX_CAPTURE_SIZE;
    }

    /// This function is called at boot time to clean up any leftover files from a capture
    /// that was previously in progress but which got interrupted. Such interruptions can be
    /// from power cycling the robot before STOP is pressed, or spontaneous reboots. In such
    /// cases, the Sidekick capture data can't be recovered as too much data was held in memory
    /// and thereby lost. However, Logcat data is still available, although it might not be
    /// fully complete if the file system did not have had time to save the latest data.
    // @@@ Respect retentionDays period!
    static void cleanupCaptureSubdirectory() {
        // Create the Sidekick subdirectory if necessary:
        File subdirectory = new File(Sidekick.SUBDIRECTORY);
        if (!subdirectory.exists()) {
            if (!subdirectory.mkdirs()) {
                Sidekick.logE("Couldn't create subdirectory");
            }
        }

        // Get the newest file in the Persist.SUBDIRECTORY directory, if any, that ends in ".logcat":
        File logcatFile = null;
        File[] fileList = new File(Sidekick.SUBDIRECTORY).listFiles();
        if (fileList != null) {
            for (File file: fileList) {
                if (file.getName().endsWith(".logcat")) {
                    if (logcatFile == null) {
                        logcatFile = file;
                    } else if (file.lastModified() > logcatFile.lastModified()) {
                        logcatFile = file;
                    }
                }
            }

            // Create a tiny capture file that contains only a LOGCAT section, and within
            // that only a LOGCAT subsection:
            if ((logcatFile != null) && (logcatFile.exists()) && (logcatFile.isFile())) {
                // Before creating a new capture, delete any old epoch ones:
                deleteEpochCaptures();

                Section[] sections = new Section[Section.COUNT];
                for (int i = 0; i < Section.COUNT; i++) {
                    sections[i] = new Section(0, 0);
                }

                // Initialize the subsection header:
                int logcatFileLength = (int) logcatFile.length();
                ByteBuffer subsectionHeader = ByteBuffer.allocate(8);
                subsectionHeader.putInt(Signature.LOGCAT); // Logcat subsection within the archive section
                subsectionHeader.putInt(logcatFileLength);
                subsectionHeader.flip();

                // Create the capture header contents with the LOGCAT subsection header positioned
                // immediately following the capture header:
                int headerSize = Header.SIZE;
                sections[Section.LOGCAT] = new Section(headerSize, subsectionHeader.limit() + logcatFileLength);
                ByteBuffer captureHeader = Header.create(
                    1 << MajorError.INTERRUPTED_CAPTURE.ordinal(), 0, 0, sections);

                // For the capture file, use the same name as 'logcatFile' but with the extension
                // ".sidekick" instead of ".temporary":
                String captureFileName = logcatFile.getName().replaceAll(
                        "\\.logcat$", " Incomplete.sidekick");
                File captureFile = new File(Sidekick.SUBDIRECTORY, captureFileName);
                try {
                    try (FileOutputStream outStream = new FileOutputStream(captureFile);
                         FileInputStream inStream = new FileInputStream(logcatFile);
                         FileChannel outChannel = outStream.getChannel();
                         FileChannel in = inStream.getChannel()) {

                        outChannel.write(captureHeader);
                        outChannel.write(subsectionHeader);
                        in.transferTo(0, logcatFileLength, outChannel); // Append the Logcat file
                        FileWorker.syncBeforeClose(outStream); // Flush to storage part 1
                    }
                    FileWorker.syncAfterClose(captureFile); // Flush to storage part 2
                } catch (IOException ignored) {
                    Sidekick.logE("Couldn't save Logcat");
                }
            }

            // Now delete all of the files in the Sidekick subdirectory that end in ".temporary"
            // and ".logcat":
            for (File file: fileList) {
                if (file.getName().endsWith(".temporary") || file.getName().endsWith(".logcat")) {
                    file.delete();
                }
            }
        }
    }
}
