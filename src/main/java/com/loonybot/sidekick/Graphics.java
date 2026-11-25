/// Sidekick graphics visualization logic.
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
    @SuppressWarnings("unused")
    static final byte RESERVED = 0;
    static final byte SET_COORDINATE_SPACE = 1;
    static final byte SET_NULL_BRUSH = 2;
    static final byte SET_BRUSH_COLOR = 3;
    static final byte SET_NULL_PEN = 4;
    static final byte SET_PEN_COLOR = 5;
    static final byte SET_PEN_WIDTH = 6;
    static final byte SET_OPACITY = 7;
    static final byte SCALE = 8;
    static final byte ROTATE = 9;
    static final byte TRANSLATE = 10;
    static final byte DRAW_TEXT = 11;
    static final byte DRAW_ELLIPSE = 12;
    static final byte DRAW_POLYGON = 13;
    static final byte DRAW_POLYLINE = 14;
    static final byte DRAW_LINE = 15;
    static final byte DRAW_RECT = 16;
}

/**
 * Graphics class for visualizations in Sidekick. Thanks to FTC Dashboard for the model.
 */
@SuppressWarnings({"unused", "UnusedReturnValue"})
public class Graphics {
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
    boolean submitted = false; // True if submitted to the capture

    /// Package private constructor. Call [Sk#createGraphics()] instead to create the object.
    Graphics() {}

    /// This package private constructor converts a Road Runner canvas to a Sidekick graphics object.
    /// We do **everything** with reflection:
    /// - Most of the FTC Dashboard canvas fields we need to access are marked `private`, and we
    ///   can work around that with reflection.
    /// - We can avoid bloating our library by having to include the FTC Dashboard library.
    /// - We can avoid taking a version dependency on the FTC Dashboard library.
    ///
    /// See [[FTC Dashboard's Field.js](https://github.com/acmerobotics/ftc-dashboard/blob/master/client/src/components/views/FieldView/Field.js)]
    /// for reference.
    Graphics(Object dashboardCanvas) {
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
                        setScale((double) op.f("scaleX"), (double) op.f("scaleY"));
                        break;
                    case "Rotation":
                        setRotation(Math.toDegrees((double) op.f("rotation")));
                        break;
                    case "Translate":
                        setTranslation((double) op.f("x"), (double) op.f("y"));
                        break;
                    case "Text":
                        // drawText basically uses only the pen for the text color:
                        if ((boolean) op.f("stroke")) {
                            setPen(canvasPen);
                        } else {
                            setPen(canvasBrush);
                        }
                        boolean usePageFrame = (boolean) op.f("usePageFrame");
                        if (usePageFrame) {
                            setCoordinateSpace(CoordinateSpace.WINDOW);
                        }
                        // Note that 'stroke' is ignored...
                        drawText((String) op.f("text"),
                                (double) op.f("x"), (double) op.f("y"),
                                (String) op.f("font"), (double) op.f("theta"));
                        if (usePageFrame) {
                            setCoordinateSpace(CoordinateSpace.FIELD);
                        }
                        break;
                    case "Circle":
                        if ((boolean) op.f("stroke")) {
                            setBrush(null);
                            setPen(canvasPen);
                        } else {
                            setBrush(canvasBrush);
                            setPen(null);
                        }
                        drawEllipse((double) op.f("x"), (double) op.f("y"),
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
                        break; // Not supported
                    case "Grid":
                        break; // TODO: Add support
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

    /// Throws if the graphics has already been submitted.
    void ensureOpen() {
        if (submitted) {
            throw new IllegalStateException("Graphics has already been submitted");
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

    //----------------------------------------------------------------------------------------------
    // Public APIs

    public Graphics setCoordinateSpace(CoordinateSpace space) {
        ensureOpen();
        if ((bufferRemaining -= 2) >= 0) {
            buffer.put(PaintId.SET_COORDINATE_SPACE);
            buffer.put((byte) space.ordinal());
        }
        return this;
    }

    /**
     * Set the brush color.
     *
     * @param color Can be null or a string like "#ffbc00".
     * @return This graphics object for chained calls.
     */
    public Graphics setBrush(String color) {
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
     * @return This graphics object for chained calls.
     */
    public Graphics setPen(String color) {
        ensureOpen();
        if (!Objects.equals(color, penColor)) {
            penColor = color;
            setColor(color, PaintId.SET_PEN_COLOR, PaintId.SET_NULL_PEN);
        }
        return this;
    }

    /**
     * Scale the current transform. Replaces any previously set scaling. Applied after
     * translation.
     *
     * @param sx x-factor scale.
     * @param sy y-factor scale.
     * @return This graphics object for chained calls.
     */
    public Graphics setScale(double sx, double sy) {
        ensureOpen();
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.SCALE);
            buffer.putFloat((float) sx);
            buffer.putFloat((float) sy);
        }
        return this;
    }

    /**
     * Rotate the current transform. Replaces any previously set rotation. Applied after
     * translation.
     *
     * @param angleInDegrees Angle in degrees.
     * @return This graphics object for chained calls.
     */
    public Graphics setRotation(double angleInDegrees) {
        ensureOpen();
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.ROTATE);
            buffer.putFloat((float) angleInDegrees);
        }
        return this;
    }

    /**
     * Set an offset to be added to every world space coordinate. Replaces any previously set
     * translation. Translation is applied after rotation and scaling is done, so it is always
     * specified in un-rotated, un-scaled field coordinates.
     *
     * @param x x offset in field coordinates.
     * @param y y offset in field coordinates.
     * @return This graphics object for chained calls.
     */
    public Graphics setTranslation(double x, double y) {
        ensureOpen();
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.TRANSLATE);
            buffer.putFloat((float) x);
            buffer.putFloat((float) y);
        }
        return this;
    }

    /**
     * Draw an ellipse.
     *
     * @param x Center of the ellipse in world space.
     * @param y Center of the ellipse in world space.
     * @param rx Radius in x-direction.
     * @param ry Radius in y-direction.
     * @return This graphics object for chained calls.
     */
    public Graphics drawEllipse(double x, double y, double rx, double ry) {
        ensureOpen();
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_ELLIPSE);
            buffer.putFloat((float) x);
            buffer.putFloat((float) y);
            buffer.putFloat((float) rx);
            buffer.putFloat((float) ry);
        }
        return this;
    }

    /**
     * Draw a polygon.
     *
     * @param points Array of points in world space.
     * @return This graphics object for chained calls.
     */
    public Graphics drawPolygon(PointD[] points) {
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
     * @return This graphics object for chained calls.
     */
    public Graphics drawPolyline(PointD[] points) {
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
     * @return This graphics object for chained calls.
     */
    public Graphics drawLine(double x1, double y1, double x2, double y2) {
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
     * @return This graphics object for chained calls.
     */
    public Graphics drawRectangle(double x, double y, double width, double height) {
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
     * Draw simple text.
     *
     * @param text Text to be drawn.
     * @param x Lower-left start position of the text, in page space.
     * @param y Lower-left start position of the text, in page space.
     * @return This graphics object for chained calls.
     */
    public Graphics drawText(String text, double x, double y) {
        return drawText(text, x, y, "", 0);
    }

    /**
     * Draw text with advanced controls.
     *
     * @param text Text to be drawn.
     * @param x Lower-left start position of the text.
     * @param y Lower-left start position of the text
     * @param font Font in HTML Canvas font format, e.g.: "italic bold 12px Arial"
     * @param theta Rotation of the text, in degrees.
     * @return This graphics object for chained calls.
     */
    public Graphics drawText(String text, double x, double y, String font, double theta) {
        ensureOpen();
        byte[] stringEncoding = text.getBytes(UTF8);
        byte[] fontEncoding = font.getBytes(UTF8);
        if ((bufferRemaining -= (14 + stringEncoding.length + 1 + fontEncoding.length + 1)) >= 0) {
            buffer.put(PaintId.DRAW_TEXT);
            buffer.putFloat((float) x);
            buffer.putFloat((float) y);
            buffer.put(stringEncoding);
            buffer.put(NULL_TERMINATOR);
            buffer.putFloat((float) theta);
            buffer.put(fontEncoding);
            buffer.put(NULL_TERMINATOR);
        }
        return this;
    }

    /**
     * Set the pen width.
     *
     * @param width Width of the pen, in world space pixels.
     * @return This graphics object for chained calls.
     */
    public Graphics setPenWidth(double width) {
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
     * @return This graphics object for chained calls.
     */
    public Graphics setOpacity(double opacity) {
        ensureOpen();
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.SET_OPACITY);
            buffer.putFloat((float) opacity);
        }
        return this;
    }

    /**
     * Submit the accumulated graphics commands to be recorded into the capture.
      */
    public void submit() {
        if (submitted)
            return;
        if (Capture.instance != null) {
            Capture.instance.recordGraphics(this);
        }
        submitted = true;
    }
}