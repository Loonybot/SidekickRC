/// Socket logic for managing Sidekick communication with the PC.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;

/// Sidekick's web socket. Each connection gets its own instance.
///
/// Protocol
/// --------
/// PC -> Robot:  JSON text frames only.
/// Robot -> PC:  JSON text frames for control/response; binary frames for download chunks.
@SuppressWarnings({"SameParameterValue", "BooleanMethodIsAlwaysInverted"})
class Socket extends NanoWSD.WebSocket {
    static final int CHUNK_SIZE = 64 * 1024; // Maximum chunk size for binary transmissions

    // Global state:
    static final Object lock = new Object(); // Lock for all sockets
    static final HashSet<Socket> allSockets = new HashSet<>(); // List of all sockets

    // Instance state:
    volatile boolean lostConnection = false;  // True when the connection has terminated
    volatile boolean cancelRequested = false; // Set by download_cancel; checked between chunks

    public Socket(NanoHTTPD.IHTTPSession handshakeRequest) {
        super(handshakeRequest);
        Sidekick.logI("Socket created!");
        synchronized(lock) {
            allSockets.add(this);
        }
    }

    @Override protected void onOpen() {}
    @Override protected void onPong(NanoWSD.WebSocketFrame pong) {}
    @Override protected void onException(IOException exception) {
        Sidekick.logE("Socket exception: %s", exception);
    }
    @Override protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
        synchronized(lock) {
            allSockets.remove(this);
        }
        lostConnection = true;
        Realtime.updateSubscription(this, new String[0]); // Unsubscribe
        Sidekick.logI("Socket closed! code=%s, reason='%s', initiatedByRemote=%b", code, reason, initiatedByRemote);
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    /// Send a JSON text frame. Silently drops on IO error (connection is already closing).
    void sendJson(String json) {
        try {
            send(json);
        } catch (IOException ignored) {}
    }

    /// Send a binary frame. Silently drops on IO error (connection is already closing).
    void sendBytes(byte[] bytes) {
        try {
            send(bytes);
        } catch (IOException ignored) {}
    }

    /// Send a JSON text frame built from a JsonObject.
    void sendJson(JsonObject obj) {
        sendJson(Sidekick.gson.toJson(obj));
    }

    /// Decode payloads with default values.
    String payloadGet(JsonObject payload, String key, String defaultValue) {
        return payload.has(key) ? payload.get(key).getAsString() : defaultValue;
    }
    boolean payloadGet(JsonObject payload, String key, boolean defaultValue) {
        return payload.has(key) ? payload.get(key).getAsBoolean() : defaultValue;
    }
    long payloadGet(JsonObject payload, String key, long defaultValue) {
        return payload.has(key) ? payload.get(key).getAsLong() : defaultValue;
    }

    /// Get a list of Sidekick files, sorted by year.
    static List<File> getCaptureFiles() {
        File directory = new File(Sidekick.SUBDIRECTORY);
        FilenameFilter filter = (dir, name) -> name.endsWith(".sidekick");
        File[] array = directory.listFiles(filter);
        if (array == null) {
            array = new File[0];
        }

        // Convert from an array to a list:
        List<File> list = new LinkedList<>(Arrays.asList(array)); // Copy the list to modify it

        // Sort by name:
        list.sort(Comparator.comparing(File::getName));

        // Move a capture that starts with "1969" or "1970" from the top of the list to the
        // bottom, because our save code guarantees that it will be the newest:
        if (list.size() > 1) { // Loop will crash if the list has either one or zero entries
            while (true) {
                String name = list.get(0).getName();
                if (name.startsWith("1969") || name.startsWith("1970")) {
                    list.add(list.remove(0));
                } else {
                    break; // ====>
                }
            }
        }
        return list;
    }

    /// Create a response JSON object for a request.
    static JsonObject createResponse(JsonObject request) {
        JsonObject response = new JsonObject();
        response.addProperty("op", request.get("op").getAsString());
        response.addProperty("type", "bot_response");
        return response;
    }

    /// Create an asynchronous message to send to the PC.
    static JsonObject createPush(String op) {
        JsonObject message = new JsonObject();
        message.addProperty("type", "bot_push");
        message.addProperty("op", op);
        return message;
    }

    // ---------------------------------------------------------------------------------------------
    // Message handlers (called from onMessage on NanoWSD's I/O thread)
    // ---------------------------------------------------------------------------------------------

    /// Tell the PC what the library version is -- and to handle the case where the library might
    /// be a newer version than the app, tell the app our minimum app version.
    void handleGetVersions(JsonObject request) {
        JsonObject response = createResponse(request);
        int ignoredAppVersion = (int) payloadGet(request, "app_version", 0);

        response.addProperty("library_version", Capture.LIBRARY_VERSION);
        response.addProperty("required_app_version", Capture.SOCKET_MIN_APP_VERSION);
        sendJson(response);
    }

    /// Get the name of the newest capture. If `terminate` is true and an opMode is currently
    /// running, stop it.
    void handleGetNewestCaptureName(JsonObject request) {
        JsonObject response = createResponse(request);
        boolean terminate = payloadGet(request, "terminate", false);

        // Check to see if an opMode is actually running - that would be the newest:
        FtcEventLoop eventLoop = Sidekick.instance.eventLoop;
        OpMode activeOpMode = eventLoop.getOpModeManager().getActiveOpMode();
        if ((activeOpMode != null) && !(activeOpMode instanceof OpModeManagerImpl.DefaultOpMode)) {
            // An opMode is running...
            if (!terminate) {
                // An opMode is still running but the caller hasn't asked us to terminate it:
                response.addProperty("name", "");
                response.addProperty("running", true);
                sendJson(response);
                return; // ===>
            } else {
                // Request that the active opMode be stopped:
                eventLoop.requestOpModeStop(activeOpMode);

                // Wait on the capture file to be closed, with a 5 second timeout. Note that if
                // the capture has already been ended (e.g., because it hit 120 seconds), the
                // latch will already be closed.
                try {
                    // Note that we intentionally don't hold any lock here:
                    Capture.instance.fileDoneLatch.await(5000, TimeUnit.MILLISECONDS);
                } catch (InterruptedException ignored) {}
            }
        }

        // Query the file directory to get a sorted list of Sidekick capture files:
        List<File> files = getCaptureFiles();

        // The newest is the last file in the sorted list:
        String newestName = (!files.isEmpty()) ? files.get(files.size() - 1).getName() : "";
        response.addProperty("name", newestName);
        response.addProperty("running", false);
        sendJson(response);
    }

    /// Get a list of Sidekick files.
    void handleGetCaptureList(JsonObject request) {
        JsonObject response = createResponse(request);

        JsonArray fileArray = new JsonArray();
        for (File file: getCaptureFiles()) {
            String supplementBaseName = FileWorker.getDateAndTimeFromCaptureName(file.getName());
            File limelightFile = new File(Sidekick.SUBDIRECTORY + "/" + supplementBaseName + ".lvc");
            File screenFile = new File(Sidekick.SUBDIRECTORY + "/" + supplementBaseName + ".mp4");

            JsonObject entry = new JsonObject();
            entry.addProperty("name", file.getName());
            entry.addProperty("archive_size", file.length());
            entry.addProperty("supplement_size", limelightFile.length() + screenFile.length());
            fileArray.add(entry);
        }

        File directory = new File(Sidekick.SUBDIRECTORY);
        response.addProperty("total_bytes", directory.getTotalSpace());
        response.addProperty("free_bytes", directory.getFreeSpace());
        response.add("files", fileArray);
        sendJson(response);
    }

    /// Return the header of a capture file. This is separate from handleDownloadData() primarily
    /// so that this can return the version number and the app can choose not to download the
    /// data.
    void handleDownloadHeader(JsonObject request) {
        cancelRequested = false; // Reset any stale cancel from a previous download

        JsonObject response = createResponse(request);
        String filename = payloadGet(request, "filename", ""); // With .sidekick extension
        long pcUnixTime = payloadGet(request, "pc_unix_time", 0L);

        Sidekick.logI("handleDownloadHeader: %s", filename); // @@@

        File captureFile = new File(Sidekick.SUBDIRECTORY, filename);
        Header header = Header.read(captureFile);
        if (header == null) {
            Sidekick.logE("Cannot open file for handleDownloadHeaderRequest: %s", filename);
            response.addProperty("error", "Cannot open file: " + filename);
            sendJson(response);
            return; // ===>
        }

        // 1. Build and send the response JSON:
        response.addProperty("clock_correction", pcUnixTime - System.currentTimeMillis());
        sendJson(response);

        // 2. Send the header as byte data:
        sendBytes(header.bytes);

        // 3. Send a done message:
        JsonObject done = createPush("download_done"); // Same op for all download types
        sendJson(done);
    }

    /// Return the data of a capture file. Sends a download response JSON, then blasts binary
    /// chunks, then sends download_done.
    void handleDownloadData(JsonObject request) {
        cancelRequested = false; // Reset any stale cancel from a previous download

        JsonObject response = createResponse(request);
        String filename = payloadGet(request, "filename", ""); // With .sidekick extension
        int[] sections = request.has("sections")
                ? Sidekick.gson.fromJson(request.get("sections"), int[].class)
                : new int[0];

        Sidekick.logI("handleDownloadData: %s", filename); // @@@

        byte[] chunkBuffer = new byte[CHUNK_SIZE]; // Kind of big so allocate here once and reuse

        File captureFile = new File(Sidekick.SUBDIRECTORY, filename);
        Header header = Header.read(captureFile);
        if (header == null) {
            Sidekick.logE("Cannot open file for handleDownloadDataRequest: %s", filename);
            response.addProperty("error", "Cannot open file: " + filename);
            sendJson(response);
            return; // ===>
        }

        // Send the successful download_response:
        sendJson(response);

        // Blast binary chunks section by section:
        boolean success = true;
        try (RandomAccessFile file = new RandomAccessFile(captureFile, "r")) {
            for (int section : sections) { // Handle in order of the sections array
                int offset = header.sections[section].offset;
                int size = header.sections[section].size;
                if (size != 0) {
                    if (!uploadFileSection(file, offset, size, chunkBuffer)) {
                        success = false;
                        break;
                    }
                }
            }
        } catch (IOException e) {
            success = false;
        }

        // Signal end of binary stream:
        JsonObject done = createPush("download_done"); // Same op for all download types
        if (cancelRequested) {
            done.addProperty("error", "Cancelled");
        } else if (!success) {
            done.addProperty("error", "File error");
        }
        sendJson(done);
        Sidekick.logI("download_done, cancelled=%s, success=%s", cancelRequested, success);
    }

    /// Screen recordings are weird because they are invoked via ADB from the PC so we don't
    /// have a good way of knowing when they're done (or even if they were started), but we
    /// want them to be done before send them to the PC.
    @SuppressWarnings("BusyWait")
    boolean waitForScreenRecordingDone(String captureName, File screenFile) {
        final int MAX_RECORDING_TIME_MS = 180000; // Maximum screen recording time (180 seconds)
        final int SCREEN_STABLE_MS = 2000;  // Size must be unchanged this long to be done
        final int SCREEN_POLL_MS = 500;   // Polling interval

        if (!screenFile.exists()) {
            return true; // ====> No need to wait if there's no file at all
        }

        // Read the capture's header:
        Header header = Header.read(new File(Sidekick.SUBDIRECTORY, captureName + ".sidekick")); // @@@ Very inconsistent use of extension
        if (header == null) {
            return false; // ====> Whoops, something went wrong
        }

        // Screen recordings have a maximum of 180 seconds. If we're past that, assume we're
        // good to go without any waiting!
        long maxEndTime = header.startUnixTimeMs + MAX_RECORDING_TIME_MS + SCREEN_STABLE_MS;
        long timeSinceEnd = System.currentTimeMillis() - maxEndTime;
        Sidekick.logI("Time since potential end of screen recording: %,d ms", timeSinceEnd);
        if (timeSinceEnd > 0) {
            return true; // ====> No need to wait since the capture is so old
        }

        // Wait for the screen recording to be finalized. The ADB screenrecord process is
        // invoked from the PC and writes the mp4 on the robot; we cannot know whether recording
        // was even started, so first poll for the file to appear, then wait for its size to
        // stabilize (no change for SCREEN_STABLE_MS), indicating the muxer has finished.

        // Poll waiting for the file's size to stabilize with no change for 2 seconds:
        // to finish
        long lastSize = -1;
        long stableStart = 0;
        while (!lostConnection) {
            long currentSize = screenFile.length();
            long now = System.currentTimeMillis();
            if (currentSize != lastSize) {
                lastSize = currentSize;
                stableStart = now;
            } else if (now - stableStart >= SCREEN_STABLE_MS) {
                break; // Size unchanged long enough — recording is complete
            }
            try { Thread.sleep(SCREEN_POLL_MS); } catch (InterruptedException ignored) {}
        }
        return true; // Success!
    }

    /// Download supplemental files. This is handled separately from handleDownloadData()
    /// primarily so that we can wait for the supplemental files to be closed.
    void handleDownloadSupplements(JsonObject request) {
        boolean success = true;
        JsonObject response = createResponse(request);
        String captureName = payloadGet(request, "filename", ""); // No file extension

        // The supplements start recording their files before we know the final capture name.
        // Their names are simply the 'dateAndTime' string. The final capture name begins with
        // 'dateAndTime' but then adds other information, such as the run number and run time.
        // Derive the original supplement base name from the final capture name:
        String supplementBaseName = FileWorker.getDateAndTimeFromCaptureName(captureName);

        Sidekick.logI("Converted '%s' to '%s':", captureName, supplementBaseName); // @@@

        File limelightFile = new File(Sidekick.SUBDIRECTORY + "/" + supplementBaseName + ".lvc");
        File screenFile = new File(Sidekick.SUBDIRECTORY + "/" + supplementBaseName + ".mp4");

        // We have to wait for screen recording but not Limelight recording to be done because
        // the former is weird while the latter is handled in normal capture termination.
        if (!waitForScreenRecordingDone(captureName, screenFile)) {
            success = false;
        }

        int limelightSize = (limelightFile.exists()) ? (int) limelightFile.length() : 0;
        int screenSize = (screenFile.exists()) ? (int) screenFile.length() : 0;

        Sidekick.logI("file name: %s, limelightSize: %d, screenSize: %d", limelightFile.getName(), limelightSize, screenSize); // @@@

        JsonArray sizeArray = new JsonArray();
        sizeArray.add(limelightSize);
        sizeArray.add(screenSize);
        response.add("sizes", sizeArray);
        sendJson(response);

        byte[] chunkBuffer = new byte[CHUNK_SIZE]; // Kind of big so allocate here once and reuse
        cancelRequested = false; // Reset any stale cancel from a previous download

        if (limelightSize != 0) {
            try (RandomAccessFile file = new RandomAccessFile(limelightFile, "r")) {
                if (!uploadFileSection(file, 0, limelightSize, chunkBuffer)) {
                    success = false;
                }
            } catch (IOException ignored) { success = false; }
        }
        if (screenSize != 0) {
            try (RandomAccessFile file = new RandomAccessFile(screenFile, "r")) {
                if (!uploadFileSection(file, 0, screenSize, chunkBuffer)) {
                    success = false;
                }
            } catch (IOException ignored) { success = false; }
        }

        // Signal end of binary stream:
        JsonObject done = createPush("download_done"); // Same op for all download types
        if (cancelRequested) {
            done.addProperty("error", "Cancelled");
        } else if (!success) {
            done.addProperty("error", "File error");
        }
        sendJson(done);
    }

    /// Blast a section of a file as a sequence of binary frames. Each frame: CHUNK_SIZE bytes
    /// of binary chunk data. Checks lostConnection and cancelRequested between chunks.
    boolean uploadFileSection(RandomAccessFile file, int offset, int size, byte[] chunkBuffer) throws IOException {
        int thisSize;
        int bytesRemaining = size;

        file.seek(offset);
        while ((thisSize = Math.min(bytesRemaining, CHUNK_SIZE)) != 0) {
            // Abandon early if a cancel request comes in (delivered by a different thread):
            if ((lostConnection) || (cancelRequested)) {
                return false; // ====>
            }
            file.readFully(chunkBuffer, 0, thisSize);

            /// [#send] takes only a byte array, so allocate a temporary array if not full chunk:
            sendBytes(thisSize == CHUNK_SIZE ? chunkBuffer : Arrays.copyOfRange(chunkBuffer, 0, thisSize));
            bytesRemaining -= thisSize;
        }
        return true; // Success!
    }

    /// Restore a configuration file, so long as the name isn't already taken.
    void handleRestoreConfiguration(JsonObject request) {
        JsonObject response = createResponse(request);
        String configName = payloadGet(request, "filename", "");
        String contents = payloadGet(request, "contents", "");

        File configFile = new File(Sidekick.SD_CARD_PATH + "/FIRST/" + configName + ".xml");
        if (configFile.exists()) {
            response.addProperty("error", "File already exists");
            sendJson(response);
            return; // ===>
        }

        // Write the contents to the file:
        try {
            try (RandomAccessFile randomAccessFile = new RandomAccessFile(configFile, "rw")) {
                randomAccessFile.write(contents.getBytes(StandardCharsets.UTF_8));
            }
        } catch (IOException e) {
            response.addProperty("error", "Failed to write file: " + e.getMessage());
        }

        sendJson(response);
    }

    /// Restart the ADB daemon.
    void handleResetAdb(JsonObject request) {
        // The WebSocket is independent of ADB and stays up throughout.
        JsonObject resetAdbResponse = createResponse(request);
        try {
            Runtime.getRuntime().exec("setprop ctl.restart adbd");
        } catch (IOException ignored) {}
        sendJson(resetAdbResponse);
    }

    /// Return a list of available realtime counters.
    void handleQueryRealtimeCounters(JsonObject request) {
        JsonObject response = createResponse(request);

        // Return an array of string pairs (name and description) for all available realtime counters:
        LinkedList<RealtimeDescriptor> descriptors = Realtime.getSupportedCounters();
        JsonArray counters = new JsonArray();
        for (RealtimeDescriptor descriptor: descriptors) {
            JsonObject counter = new JsonObject();
            counter.addProperty("name", descriptor.name);
            counter.addProperty("description", descriptor.description);
            counters.add(counter);
        }
        response.add("counters", counters);
        response.addProperty("update_interval",SampleWorker.REALTIME_UPDATE_PERIOD);
        sendJson(response);
    }

    /// Subscribe to the requested realtime counters; unsubscribe if the list is empty.
    void handleSubscribeRealtime(JsonObject request) {
        JsonObject response = createResponse(request);
        String[] names = request.has("names")
                ? Sidekick.gson.fromJson(request.get("names"), String[].class)
                : new String[0];

        Realtime.updateSubscription(this, names);
        sendJson(response);
    }

    /// Push realtime counter data to the PC.
    void pushRealtimeCounters(Set<String> subscribedCounters, HashMap<String, List<Float>> sampleData) {
        JsonObject message = createPush("realtime_sample");
        JsonArray counters = new JsonArray();
        for (String subscribedCounter: subscribedCounters) {
            List<Float> sample = sampleData.get(subscribedCounter);
            if (sample != null) {
                JsonArray values = new JsonArray();
                for (Float value: Objects.requireNonNull(sampleData.get(subscribedCounter))) {
                    values.add(value);
                }
                JsonObject counter = new JsonObject();
                counter.addProperty("name", subscribedCounter);
                counter.add("values", values);
                counters.add(counter);
            }
        }
        message.add("counters", counters);
        sendJson(message);
    }

    /// Broadcast a message to all connected PCs to let them know that Start has just been
    /// pressed on the Driver Station and they can start screen recording if they have an ADB
    /// connection. `timeLimit` is in seconds.
    static void broadcastStartScreenRecord(String fileName, boolean usingCamera, int timeLimit) {
        JsonObject message = createPush("start_screen_record");
        message.addProperty("file_name", fileName);
        message.addProperty("time_limit", timeLimit);
        message.addProperty("using_camera", usingCamera);
        synchronized(lock) {
            for (Socket socket: allSockets) {
                socket.sendJson(message);
            }
        }
    }

    /// Broadcast a message to all connected PCs to let them know that Stop has just been
    /// pressed and they should stop screen recording if they were recording.
    static void broadcastStopScreenRecord() {
        JsonObject message = createPush("stop_screen_record");
        synchronized(lock) {
            for (Socket socket: allSockets) {
                socket.sendJson(message);
            }
        }
    }

    /// Broadcast the minimum battery voltage read by any hub since our last broadcast.
    /// In addition to providing useful information for the app to display, this also
    /// acts as a "keep-alive" for the network connection.
    static void broadcastBatteryVoltage() {
        synchronized (lock) {
            int minMv = Capture.minBatteryMilliVolts.getAndUpdate(v -> Integer.MAX_VALUE);
            int lastMv = Capture.lastBatteryMilliVolts;

            // Send the minimum voltage read since our last broadcast; if no new data has been read,
            // push the most recent read:
            int mv = (minMv < Integer.MAX_VALUE) ? minMv : lastMv;

            JsonObject message = createPush("battery_voltage");
            message.addProperty("millivolts", mv);
            for (Socket socket: allSockets) {
                socket.sendJson(message);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // NanoWSD message dispatch
    // ---------------------------------------------------------------------------------------------

    /// Called by NanoWSD for every incoming WebSocket frame.
    /// All PC to Robot messages are JSON text frames; binary frames from PC are ignored.
    @Override protected void onMessage(NanoWSD.WebSocketFrame frame) {
        if (frame.getOpCode() != NanoWSD.WebSocketFrame.OpCode.Text) {
            Sidekick.logW("Unexpected binary frame from PC — ignoring");
            return;
        }

        String text = frame.getTextPayload();
        Sidekick.logI("onMessage: %.240s", text);

        JsonObject payload;
        try {
            payload = new JsonParser().parse(text).getAsJsonObject();
        } catch (Exception e) {
            Sidekick.logW("Malformed JSON from PC.");
            return;
        }

        String type = payloadGet(payload, "type", "");
        String op = payloadGet(payload, "op", "");
        if (type.equals("download_cancel")) {
            cancelRequested = true;  // Checked between chunks in downloadSection
        } else if (type.equals("pc_request")) {
            switch (op) {
                // ---------------------------------------------------------------------------------
                // Request/response handlers
                // ---------------------------------------------------------------------------------
                case "get_versions":
                    handleGetVersions(payload);
                    break;
                case "get_newest_capture_name":
                    handleGetNewestCaptureName(payload);
                    break;
                case "get_capture_list":
                    handleGetCaptureList(payload);
                    break;
                case "download_header":
                    handleDownloadHeader(payload);
                    break;
                case "download_data":
                    handleDownloadData(payload);
                    break;
                case "download_supplements":
                    handleDownloadSupplements(payload);
                    break;
                case "restore_configuration":
                    handleRestoreConfiguration(payload);
                    break;
                case "reset_adb":
                    handleResetAdb(payload);
                    break;
                case "query_realtime_counters":
                    handleQueryRealtimeCounters(payload);
                    break;
                case "subscribe_realtime":
                    handleSubscribeRealtime(payload);
                    break;
                default:
                    Sidekick.logW("Unknown message op from PC: %s", op);
            }
        } else {
            Sidekick.logW("Unknown message type from PC: %s", type);
        }
    }
}