/// Common interface for all objects wrapped by the Sidekick proxy builder.
///
/// Copyright James Goossen.
package com.loonybot.sidekick;

import android.graphics.Color;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Objects;

/// Paint command identifiers.
class PaintId {
    static final byte SET_NULL_BRUSH = 1;
    static final byte SET_BRUSH_COLOR = 2;
    static final byte SET_NULL_PEN = 3;
    static final byte SET_PEN_COLOR = 4;
    static final byte SET_PEN_WIDTH = 5;
    static final byte SET_OPACITY = 6;
    static final byte SCALE = 7;
    static final byte ROTATE = 8;
    static final byte TRANSLATE = 9;
    static final byte DRAW_TEXT = 10;
    static final byte DRAW_ELLIPSE = 11;
    static final byte DRAW_POLYGON = 12;
    static final byte DRAW_POLYLINE = 13;
    static final byte DRAW_LINE = 14;
    static final byte DRAW_RECT = 15;
}

/**
 * Canvas class for visualizations in Sidekick. Thanks to FTC Dashboard for the model.
 */
@SuppressWarnings({"unused", "UnusedReturnValue"})
public class Canvas implements AutoCloseable {
    /// Helper class for converting FTC Dashboard CanvasOps.
    class DashboardCanvasOp {
        Object op; // Instance of a class that extends com.acmerobotics.dashboard.canvas.CanvasOp
        Class<?> opClass; // Class type of 'op'
        DashboardCanvasOp(Object op) {
            this.op = op;
            this.opClass = op.getClass();
        }

        /// Return the root of the class's type name.
        String getName() {
            String fullName = opClass.getName();
            int index = fullName.lastIndexOf('$');
            if (index == -1) {
                index = fullName.lastIndexOf('.');
            }
            if ((index == -1) || (!fullName.startsWith("com.acmerobotics.dashboard.canvas."))) {
                capture.minorError("Expected a dashboard type in the canvas, found %s", fullName);
                return ""; // ====>
            }
            return fullName.substring(index + 1);
        }

        /// Retrieve a field of the class.
        Object f(String fieldName) throws IllegalAccessException {
            try {
                Field field = opClass.getDeclaredField(fieldName);
                field.setAccessible(true);
                return field.get(op);
            } catch (NoSuchFieldException|IllegalAccessException e) {
                capture.minorError("Can't access field '%s' in class '%s'", fieldName, opClass.getName());
                throw new IllegalAccessException(); // ====>
            }
        }
    }

    final int BUFFER_SIZE = 4*1024; // Must be less than 32K because 16 bits is used for size in bytes
    final Charset UTF8 = StandardCharsets.UTF_8; // Our standard charset for strings
    final byte NULL_TERMINATOR = 0; // We null-terminate all strings in the log

    Capture capture = Capture.instance; // Our capture object
    ByteBuffer buffer = ByteBuffer.allocate(BUFFER_SIZE); // Our one-and-only buffer
    int bufferRemaining = BUFFER_SIZE; // Amount of buffer space remaining, negative if ran out
    String brushColor; // Currently set brush color
    String penColor; // Currently set pen color
    boolean submitted = false; // True if the canvas has been submitted to the capture

    /// Package private constructor. Call [Sk#createCanvas()] instead to create a new canvas.
    Canvas() {}

    /// This package private constructor converts a Road Runner canvas to a Sidekick canvas object.
    /// We do **everything** with reflection:
    /// - Most of the FTC Dashboard canvas fields we need to access are marked `private`, and we
    ///   can work around that with reflection.
    /// - We can avoid bloating our library by having to include the FTC Dashboard library.
    /// - We can avoid taking a version dependency on the FTC Dashboard library.
    ///
    /// See [[FTC Dashboard's Field.js](https://github.com/acmerobotics/ftc-dashboard/blob/master/client/src/components/views/FieldView/Field.js)]
    /// for reference.
    Canvas(Object dashboardCanvas) {
        if ((dashboardCanvas == null) || (!dashboardCanvas.getClass().getName().equals("com.acmerobotics.dashboard.canvas.Canvas"))) {
            throw new IllegalArgumentException("Sidekick: Must be a com.acmerobotics.dashboard.canvas Canvas object.");
        }

        List<?> ops = (List<?>) Sidekick.getInstanceField(dashboardCanvas, "ops", List.class);
        if (ops == null) {
            capture.minorError("Unexpected null dashboard canvas op");
            return; // ====>
        }

        // FTC Dashboard starts with the JavaScript default brush and pen which are both black:
        String canvasBrush = "#000000";
        String canvasPen = "#000000";

        for (Object opObject: ops) {
            if (opObject == null) {
                capture.minorError("Unexpected null dashboard canvas op");
                return; // ====>
            }

            // Wrap the Dashboard CanvasOp with our own DashboardCanvasOp class:
            DashboardCanvasOp op = new DashboardCanvasOp(opObject);
            String name = op.getName();
            try {
                switch (name) {
                    case "Scale":
                        scale((double) op.f("scaleX"), (double) op.f("scaleY"));
                        break;
                    case "Rotation":
                        rotate(Math.toDegrees((double) op.f("rotation")));
                        break;
                    case "Translate":
                        translate(new PointD((double) op.f("x"), (double) op.f("y")));
                        break;
                    case "Text":
                        // drawText basically uses only the pen for the text color:
                        if ((boolean) op.f("stroke")) {
                            setPen(canvasPen);
                        } else {
                            setPen(canvasBrush);
                        }
                        drawText(new PointD((double) op.f("x"), (double) op.f("y")), (String) op.f("text"),
                                (double) op.f("theta"), (boolean) op.f("stroke"), (boolean) op.f("usePageFrame"),
                                (String) op.f("font"));
                        break;
                    case "Circle":
                        if ((boolean) op.f("stroke")) {
                            setBrush(null);
                            setPen(canvasPen);
                        } else {
                            setBrush(canvasBrush);
                            setPen(null);
                        }
                        drawEllipse(new PointD((double) op.f("x"), (double) op.f("y")),
                                (double) op.f("radius"), (double) op.f("radius"));
                        break;
                    case "Polygon": {
                            if ((boolean) op.f("stroke")) {
                                setBrush(null);
                                setPen(canvasPen);
                            } else {
                                setBrush(canvasBrush);
                                setPen(null);
                            }
                            double[] xPoints = (double[]) op.f("xPoints");
                            double[] yPoints = (double[]) op.f("yPoints");
                            if (xPoints.length != yPoints.length)
                                return;
                            PointD[] points = new PointD[xPoints.length];
                            for (int i = 0; i < xPoints.length; i++) {
                                points[i] = new PointD(xPoints[i], yPoints[i]);
                            }
                            drawPolygon(points);
                        }
                        break;
                    case "Polyline": {
                            double[] xPoints = (double[]) op.f("xPoints");
                            double[] yPoints = (double[]) op.f("yPoints");
                            if (xPoints.length != yPoints.length)
                                return;
                            PointD[] points = new PointD[xPoints.length];
                            for (int i = 0; i < xPoints.length; i++) {
                                points[i] = new PointD(xPoints[i], yPoints[i]);
                            }
                            drawPolyline(points);
                        }
                        break;
                    case "Fill":
                        canvasBrush = (String) op.f("color");
                        break;
                    case "Stroke":
                        canvasPen = (String) op.f("color");
                        break;
                    case "StrokeWidth":
                        setPenWidth((int) op.f("width"));
                        break;
                    case "Image":
                    case "Grid": // TODO: Add support
                        break;
                    case "Alpha":
                        setOpacity((double) op.f("alpha"));
                        break;
                    default:
                        capture.minorError("Unexpected dashboard canvas command '%s'", name);
                        return; // ====>
                }
                assert(bufferRemaining == buffer.remaining());
            } catch (IllegalAccessException ignored){
                return; // ====> The field access error was already recorded
            }
        }
    }

    /// Throws if the canvas has already been submitted.
    void ensureOpen() {
        if (submitted) {
            throw new IllegalStateException("Canvas has already been submitted");
        }
    }

    /// Helper for setBrush() and setPen().
    void setColor(String color, byte colorId, byte nullId) {
        ensureOpen();
        if (color == null) {
            if ((bufferRemaining -= 1) >= 0) {
                buffer.put(nullId);
            }
        } else {
            if ((bufferRemaining -= 5) >= 0) {
                buffer.put(colorId);
                buffer.putInt(Color.parseColor(color));
            }
        }
    }

    /// drawText() is not supported yet.
    private Canvas drawText(PointD position, String text) {
        return drawText(position, text, 0, false, false, "");
    }

    /// drawText() is not supported yet.
    private Canvas drawText(PointD position, String text, double theta, boolean stroke, boolean usePageFrame, String font) {
        ensureOpen();
        byte[] stringEncoding = text.getBytes(UTF8);
        byte[] fontEncoding = font.getBytes(UTF8);
        if ((bufferRemaining -= (15 + stringEncoding.length + 1 + fontEncoding.length + 1)) >= 0) {
            buffer.put(PaintId.DRAW_TEXT);
            buffer.putFloat((float) position.x);
            buffer.putFloat((float) position.y);
            buffer.put(stringEncoding);
            buffer.put(NULL_TERMINATOR);
            buffer.putFloat((float) theta);
            buffer.put((byte) (stroke ? 1 : 0));
            buffer.put((byte) (usePageFrame ? 1 : 0));
            buffer.put(fontEncoding);
            buffer.put(NULL_TERMINATOR);
        }
        return this;
    }


    //----------------------------------------------------------------------------------------------
    // Public APIs

    /**
     * Set the brush color.
     *
     * @param color Can be null or a string like "#ffbc00".
     * @return This canvas for chained calls.
     */
    public Canvas setBrush(String color) {
        ensureOpen();
        if (!Objects.equals(color, brushColor)) {
            brushColor = color;
            setColor(color, PaintId.SET_BRUSH_COLOR, PaintId.SET_NULL_BRUSH);
        }
        return this;
    }

    /**
     * Set the pen color.
     *
     * @param color Can be null or a string like "#ffbc00".
     * @return This canvas for chained calls.
     */
    public Canvas setPen(String color) {
        ensureOpen();
        if (!Objects.equals(color, penColor)) {
            penColor = color;
            setColor(color, PaintId.SET_PEN_COLOR, PaintId.SET_NULL_PEN);
        }
        return this;
    }

    /**
     * Scale the current transform. Replaces any previously set scaling.
     *
     * @param sx x-factor scale.
     * @param sy y-factor scale.
     * @return This canvas for chained calls.
     */
    public Canvas scale(double sx, double sy) {
        ensureOpen();
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.SCALE);
            buffer.putFloat((float) sx);
            buffer.putFloat((float) sy);
        }
        return this;
    }

    /**
     * Rotate the current transform. Replaces any previously set rotation.
     *
     * @param angleInDegrees Angle in degrees.
     * @return This canvas for chained calls.
     */
    public Canvas rotate(double angleInDegrees) {
        ensureOpen();
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.ROTATE);
            buffer.putFloat((float) angleInDegrees);
        }
        return this;
    }

    /**
     * Set the offset for the current transform. Replaces any previously set translation.
     *
     * @param offset Offset.
     * @return This canvas for chained calls.
     */
    public Canvas translate(PointD offset) {
        ensureOpen();
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.TRANSLATE);
            buffer.putFloat((float) offset.x);
            buffer.putFloat((float) offset.y);
        }
        return this;
    }

    /**
     * Draw an ellipse.
     *
     * @param center Center of the ellipse in world space.
     * @param rx Radius in x-direction.
     * @param ry Radius in y-direction.
     * @return This canvas for chained calls.
     */
    public Canvas drawEllipse(PointD center, double rx, double ry) {
        ensureOpen();
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_ELLIPSE);
            buffer.putFloat((float) center.x);
            buffer.putFloat((float) center.y);
            buffer.putFloat((float) rx);
            buffer.putFloat((float) ry);
        }
        return this;
    }

    /**
     * Draw a polygon.
     *
     * @param points Array of points in world space.
     * @return This canvas for chained calls.
     */
    public Canvas drawPolygon(PointD[] points) {
        ensureOpen();
        if ((bufferRemaining -= (3 + 8*points.length)) >= 0) {
            if (points.length < Short.MAX_VALUE / 10) {
                buffer.put(PaintId.DRAW_POLYGON);
                buffer.putShort((short) points.length);
                for (PointD point : points) {
                    buffer.putFloat((float) point.x);
                    buffer.putFloat((float) point.y);
                }
            }
        }
        return this;
    }

    /**
     * Draw a connected list of lines.
     *
     * @param points Array of points in world space.
     * @return This canvas for chained calls.
     */
    public Canvas drawPolyline(PointD[] points) {
        ensureOpen();
        if ((bufferRemaining -= (3 + 8*points.length)) >= 0) {
            if (points.length < Short.MAX_VALUE / 10) {
                buffer.put(PaintId.DRAW_POLYLINE);
                buffer.putShort((short) points.length);
                for (PointD point : points) {
                    buffer.putFloat((float) point.x);
                    buffer.putFloat((float) point.y);
                }
            }
        }
        return this;
    }

    /**
     * Draw a line.
     *
     * @param x1 Start of line in world space.
     * @param y1 Start of line in world space.
     * @param x2 End of line in world space.
     * @param y2 End of line in world space.
     * @return This canvas for chained calls.
     */
    public Canvas drawLine(double x1, double y1, double x2, double y2) {
        ensureOpen();
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_LINE);
            buffer.putFloat((float) x1);
            buffer.putFloat((float) y1);
            buffer.putFloat((float) x2);
            buffer.putFloat((float) y2);
        }
        return this;
    }

    /**
     * Draw a rectangle.
     *
     * @param x X coordinate of the corner, in world space.
     * @param y Y coordinate of the corner, in world space.
     * @param width Width of the rectangle, in world space.
     * @param height Height of the rectangle, in world space.
     * @return This canvas for chained calls.
     */
    public Canvas drawRect(double x, double y, double width, double height) {
        ensureOpen();
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_RECT);
            buffer.putFloat((float) x);
            buffer.putFloat((float) y);
            buffer.putFloat((float) width);
            buffer.putFloat((float) height);
        }
        return this;
    }

    /**
     * Set the pen width.
     *
     * @param width Width of the pen, in world space pixels.
     * @return This canvas for chained calls.
     */
    public Canvas setPenWidth(double width) {
        ensureOpen();
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.SET_PEN_WIDTH);
            buffer.putFloat((float) width);
        }
        return this;
    }

    /**
     * Set the opacity.
     *
     * @param opacity Opacity, from 0.0 (fully transparent) to 1.0 (opaque).
     * @return This canvas for chained calls.
     */
    public Canvas setOpacity(double opacity) {
        ensureOpen();
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.SET_OPACITY);
            buffer.putFloat((float) opacity);
        }
        return this;
    }

    /**
     * Auto-closable close() method.
     */
    @Override
    public void close() {
        if (!submitted) {
            submit();
        }
    }

    /**
     * Submit the accumulated canvas commands to be recorded into the capture.
      */
    public void submit() {
        ensureOpen();
        if (Capture.instance != null) {
            Capture.instance.recordCanvas(this);
        }
        submitted = true;
    }
}
