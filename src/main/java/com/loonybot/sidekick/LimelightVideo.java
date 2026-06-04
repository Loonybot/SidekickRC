/// Thread logic for capturing Limelight video.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import static java.lang.System.nanoTime;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.RandomAccessFile;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

// Captures the MJPEG video stream from the Limelight camera to a local file during the run.
//
// File format (.lvc — Limelight Video Capture):
//
//   Header (32 bytes, placeholder at open, overwritten at close via seek(0)):
//     [4]  magic: LVC_MAGIC ("LVC\0")
//     [4]  version: LVC_VERSION
//     [4]  frame_count
//     [4]  reserved
//     [8]  start_nano_time  (nanoTime() when capture thread started; aligns with archive clock)
//     [8]  index_offset     (byte offset to the start of the index table)
//
//   Frame records (sequential from byte 32):
//     [4]  jpeg_size
//     [4]  rel_time_ms      (milliseconds since start_nano_time)
//     [jpeg_size]  JPEG bytes
//
//   Index (at index_offset, after all frame records):
//     frame_count × 12 bytes: [4B rel_time_ms][8B file_offset]
//     file_offset points to the jpeg_size field of the corresponding frame record.
//
// All multi-byte integers use Java ByteBuffer default big-endian byte order.
class LimelightVideo {
    static final int PORT = 5800;            // Port for the Limelight's MJPEG stream with visualizations
    static final int LVC_MAGIC = 0x4C564300; // "LVC\0"
    static final int LVC_VERSION = 1;
    static final int HEADER_SIZE = 32;       // [4+4+4+4+8+8] bytes
    static final int FRAME_HEADER_SIZE = 8;  // [4B jpeg_size][4B rel_time_ms]
    static final int INDEX_ENTRY_SIZE = 12;  // [4B rel_time_ms][8B file_offset]

    final String filename;
    final String address;
    volatile boolean stopRequested;
    volatile boolean hasError;
    final CountDownLatch doneLatch = new CountDownLatch(1);
    Thread captureThread;

    //------------------------------------------------------------------------------------------
    // Callable from any thread

    LimelightVideo(String filename, String address) {
        this.filename = filename;
        this.address = address;
        captureThread = new Thread(this::capture);
        captureThread.start();
    }

    /// Signal the capture thread to stop.
    void stop() {
        stopRequested = true;
        captureThread.interrupt();
    }

    //------------------------------------------------------------------------------------------
    // Capture thread

    /// The Limelight capture thread's entry point.
    void capture() {
        Sidekick.instance.registerThreadStart(Sidekick.VIDEO_CAPTURE_THREAD_NAME);

        RandomAccessFile file = null;
        FileChannel channel = null;
        ArrayList<long[]> index = new ArrayList<>();
        long captureStartNanoTime;

        Sidekick.logI("+++ Limelight video capture started");

        try {
            file = new RandomAccessFile(filename, "rw");
            channel = file.getChannel();

            // Write a placeholder header; the real values are written at close:
            ByteBuffer placeholder = ByteBuffer.allocate(HEADER_SIZE);
            placeholder.putInt(LVC_MAGIC);
            placeholder.putInt(LVC_VERSION);
            placeholder.putInt(0);          // frame_count placeholder
            placeholder.putInt(0);          // reserved
            captureStartNanoTime = nanoTime();
            placeholder.putLong(captureStartNanoTime);
            placeholder.putLong(0);         // index_offset placeholder
            placeholder.flip();
            channel.write(placeholder);

            HttpURLConnection connection = null;
            BufferedInputStream byteStream = null;
            ByteBuffer frameHeader = ByteBuffer.allocate(FRAME_HEADER_SIZE); // Reused per frame

            while ((!stopRequested) && (!Thread.currentThread().isInterrupted())) {
                try {
                    if (connection == null) {
                        connection = (HttpURLConnection) new URL("http://" + address + ":" + PORT)
                                .openConnection();
                        connection.setConnectTimeout(3000);
                        connection.setReadTimeout(3000);
                        connection.connect();
                        if (connection.getResponseCode() != 200) {
                            throw new IOException("Unexpected response: " + connection.getResponseCode());
                        }
                        byteStream = new BufferedInputStream(connection.getInputStream());
                    }

                    // MJPEG frame header format (newlines are CRLF):
                    //   --boundarydonotcross
                    //   Content-Type: image/jpeg
                    //   Content-Length: <n>
                    //   X-Timestamp: 0.000000
                    //   (blank line)
                    //   [Binary JPEG data]
                    readLine(byteStream); // --boundary
                    readLine(byteStream); // Content-Type
                    String contentLengthLine = readLine(byteStream); // Content-Length: <n>
                    readLine(byteStream); // X-Timestamp
                    readLine(byteStream); // blank line

                    String numStr = contentLengthLine.replaceAll("[^0-9]", "");
                    if (numStr.isEmpty()) {
                        hasError = true;
                        disconnect(connection);
                        connection = null;
                        byteStream = null;
                        continue;
                    }
                    int length = Integer.parseInt(numStr);

                    // Validate JPEG start marker (0xFF 0xD8):
                    byteStream.mark(2);
                    if (byteStream.read() != 0xFF || byteStream.read() != 0xD8) {
                        hasError = true;
                        disconnect(connection);
                        connection = null;
                        byteStream = null;
                        continue;
                    }
                    byteStream.reset();

                    // Read the JPEG payload:
                    byte[] jpeg = new byte[length];
                    int sum = 0;
                    while (sum < length) {
                        int n = byteStream.read(jpeg, sum, length - sum);
                        if (n < 0) throw new IOException("Unexpected end of stream");
                        sum += n;
                    }

                    // Validate JPEG end marker (0xFF 0xD9):
                    if ((jpeg[length - 2] != (byte) 0xFF) || (jpeg[length - 1] != (byte) 0xD9)) {
                        hasError = true;
                        disconnect(connection);
                        connection = null;
                        byteStream = null;
                        continue;
                    }

                    // Record file position and relative timestamp before writing:
                    long fileOffset = channel.position();
                    int relMs = (int) ((nanoTime() - captureStartNanoTime) / 1_000_000L);

                    // Write frame record: [4B jpeg_size][4B rel_time_ms][JPEG bytes]
                    frameHeader.clear();
                    frameHeader.putInt(length);
                    frameHeader.putInt(relMs);
                    frameHeader.flip();
                    channel.write(frameHeader);
                    ByteBuffer jpegBuf = ByteBuffer.wrap(jpeg);
                    while (jpegBuf.hasRemaining())
                        channel.write(jpegBuf);

                    index.add(new long[]{relMs, fileOffset});

                } catch (IOException e) {
                    if (!stopRequested) {
                        hasError = true;
                        Sidekick.logE("Limelight video capture I/O error: %s", e.getMessage());
                    }
                    disconnect(connection);
                    connection = null;
                    byteStream = null;
                    if (!stopRequested) {
                        break; // Hard failure; don't retry
                    }
                }
            }

            if (connection != null)
                disconnect(connection);

            // Append the index table:
            long indexOffset = channel.position();
            ByteBuffer indexBuf = ByteBuffer.allocate(index.size() * INDEX_ENTRY_SIZE);
            for (long[] entry : index) {
                indexBuf.putInt((int) entry[0]); // rel_time_ms
                indexBuf.putLong(entry[1]);      // file_offset
            }
            indexBuf.flip();
            channel.write(indexBuf);

            // Overwrite the placeholder header with real values:
            ByteBuffer finalHeader = ByteBuffer.allocate(HEADER_SIZE);
            finalHeader.putInt(LVC_MAGIC);
            finalHeader.putInt(LVC_VERSION);
            finalHeader.putInt(index.size());
            finalHeader.putInt(0); // reserved
            finalHeader.putLong(captureStartNanoTime);
            finalHeader.putLong(indexOffset);
            finalHeader.flip();
            channel.write(finalHeader, 0); // Positional write; does not advance channel position

            // Sync and close:
            file.getFD().sync();
            channel.close();
            file.close();

            Sidekick.logI("+++ Limelight video capture: %d frames saved to %s", index.size(), filename);

        } catch (IOException e) {
            hasError = true;
            Sidekick.logE("+++ Limelight video capture failed: %s", e.getMessage());
            if (channel != null) { try { channel.close(); } catch (IOException ignored) {} }
            if (file != null) { try { file.close(); } catch (IOException ignored) {} }
        } finally {
            doneLatch.countDown();
            Sidekick.instance.registerThreadEnd();
        }
    }

    /// Disconnect to reset the connection.
    static void disconnect(HttpURLConnection connection) {
        if (connection != null)
            connection.disconnect();
    }

    /// Read a line from the input stream, handling CRLF line endings.
    static String readLine(InputStream stream) throws IOException {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        while (true) {
            int raw = stream.read();
            if (raw == -1) throw new IOException("Unexpected end of stream");
            byte chr = (byte) raw;
            if (chr == '\n') break; // Stop on newline
            if (chr != '\r') buffer.write(chr); // Save everything but carriage returns
        }
        return buffer.toString(Charset.defaultCharset().name());
    }
}
