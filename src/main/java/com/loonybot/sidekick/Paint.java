/// Common interface for all objects wrapped by the Sidekick proxy builder.
///
/// Copyright Andrew Goossen.
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

/// API object for rendering.
/// @noinspection UnusedReturnValue
class Paint {
    class CanvasOp {
        Object op; // Instance of a class that extends com.acmerobotics.dashboard.canvas.CanvasOp
        Class<?> opClass; // Class type of 'op'
        CanvasOp(Object op) {
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
                capture.error(MinorError.CANVAS, "Expected a dashboard type in the canvas, found " + fullName);
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
                capture.error(MinorError.CANVAS, String.format("Can't access field '%s' in class '%s'",
                        fieldName, opClass.getName()));
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

    /// Public constructor.
    public Paint() {}

    /// This private constructor converts a Road Runner canvas to a Sidekick paint object. We do
    /// **everything** with reflection:
    /// - Most of the FTC Dashboard canvas fields we need to access are marked `private`, and we
    ///   can work around that with reflection.
    /// - We can avoid bloating our library by having to include the FTC Dashboard library.
    /// - We can avoid taking a version dependency on the FTC Dashboard library.
    ///
    /// See [[FTC Dashboard's Field.js](https://github.com/acmerobotics/ftc-dashboard/blob/master/client/src/components/views/FieldView/Field.js)]
    /// for reference.
    Paint(Object canvas) {
        if ((canvas == null) || (!canvas.getClass().getName().equals("com.acmerobotics.dashboard.canvas.Canvas"))) {
            throw new IllegalArgumentException("Sidekick: Must be a com.acmerobotics.dashboard.canvas Canvas object.");
        }

        List<Object> ops;
        try {
            Field field = canvas.getClass().getDeclaredField("ops");
            field.setAccessible(true);
            //noinspection unchecked
            ops = (List<Object>) field.get(canvas);
        } catch (Exception e) {
            capture.error(MinorError.CANVAS, "Inaccessible Canvas ops.");
            return; // ====>
        }

        if (ops == null) {
            capture.error(MinorError.CANVAS, "Unexpected null dashboard canvas op");
            return; // ====>
        }

        // FTC Dashboard starts with the JavaScript default brush and pen which are both black:
        String canvasBrush = "#000000";
        String canvasPen = "#000000";

        for (Object opObject: ops) {
            if (opObject == null) {
                capture.error(MinorError.CANVAS, "Unexpected null dashboard canvas op");
                return; // ====>
            }

            CanvasOp op = new CanvasOp(opObject);
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
                        capture.error(MinorError.CANVAS, "Unexpected dashboard canvas command '%s'", name);
                        return; // ====>
                }
                assert(bufferRemaining == buffer.remaining());
            } catch (IllegalAccessException ignored){
                return; // ====> The field access error was already recorded
            }
        }
    }

    /// Helper for setBrush() and setPen():
    void setColor(String color, byte colorId, byte nullId) {
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

    /// Can take null or a string like "#ffbc00".
    public Paint setBrush(String color) {
        if (!Objects.equals(color, brushColor)) {
            brushColor = color;
            setColor(color, PaintId.SET_BRUSH_COLOR, PaintId.SET_NULL_BRUSH);
        }
        return this;
    }
    public Paint setPen(String color) {
        if (!Objects.equals(color, penColor)) {
            penColor = color;
            setColor(color, PaintId.SET_PEN_COLOR, PaintId.SET_NULL_PEN);
        }
        return this;
    }
    public Paint scale(double sx, double sy) {
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.SCALE);
            buffer.putFloat((float) sx);
            buffer.putFloat((float) sy);
        }
        return this;
    }
    public Paint rotate(double angleInDegrees) {
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.ROTATE);
            buffer.putFloat((float) angleInDegrees);
        }
        return this;
    }
    public Paint translate(PointD offset) {
        if ((bufferRemaining -= 9) >= 0) {
            buffer.put(PaintId.TRANSLATE);
            buffer.putFloat((float) offset.x);
            buffer.putFloat((float) offset.y);
        }
        return this;
    }
    public Paint drawText(PointD position, String text) {
        return drawText(position, text, 0, false, false, "");
    }
    public Paint drawText(PointD position, String text, double theta, boolean stroke, boolean usePageFrame, String font) {
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
    public Paint drawEllipse(PointD center, double rx, double ry) {
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_ELLIPSE);
            buffer.putFloat((float) center.x);
            buffer.putFloat((float) center.y);
            buffer.putFloat((float) rx);
            buffer.putFloat((float) ry);
        }
        return this;
    }
    public Paint drawPolygon(PointD[] points) {
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
    public Paint drawPolyline(PointD[] points) {
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
    public Paint drawLine(double x1, double y1, double x2, double y2) {
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_LINE);
            buffer.putFloat((float) x1);
            buffer.putFloat((float) y1);
            buffer.putFloat((float) x2);
            buffer.putFloat((float) y2);
        }
        return this;
    }
    public Paint drawRect(double x, double y, double width, double height) {
        if ((bufferRemaining -= 17) >= 0) {
            buffer.put(PaintId.DRAW_RECT);
            buffer.putFloat((float) x);
            buffer.putFloat((float) y);
            buffer.putFloat((float) width);
            buffer.putFloat((float) height);
        }
        return this;
    }
    public Paint setPenWidth(double width) {
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.SET_PEN_WIDTH);
            buffer.putFloat((float) width);
        }
        return this;
    }
    public Paint setOpacity(double opacity) {
        if ((bufferRemaining -= 5) >= 0) {
            buffer.put(PaintId.SET_OPACITY);
            buffer.putFloat((float) opacity);
        }
        return this;
    }
}
