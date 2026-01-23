package org.firstinspires.ftc.teamcode.Cogintilities;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.Contract;

public enum Color implements TeamConstants {
    BLACK (0.000,      "#000000", new RgbValues(0,   0,   0  )),
    RED   (0.279, "#FF0000", new RgbValues(255, 0,   0  )),
    ORANGE(0.333, "#FFA500", new RgbValues(255, 165, 0  )),
    YELLOW(0.388, "#FFFF00", new RgbValues(255, 255, 0  )),
    SAGE  (0.444, "#BCB88A", new RgbValues(188, 184, 138)), //Note: not a great match with true color
    GREEN (0.500, "#008000", new RgbValues(0,   128, 0  )),
    AZURE (0.555, "#007FFF", new RgbValues(0,   127, 255)),
    BLUE  (0.611, "#0000FF", new RgbValues(0,   0,   255)),
    INDIGO(0.666, "#4B0082", new RgbValues(75,  0,   130)),
    VIOLET(0.722, "#7F00FF", new RgbValues(127, 0,   255)),
    WHITE (1.000, "#FFFFFF", new RgbValues(255, 255, 255));

    private final double servoValue;
    private final String hexValue;
    private final RgbValues rgbValues;

    Color(double servoValue, String hexValue, RgbValues rgbValues) {
        this.servoValue = servoValue;
        this.hexValue = hexValue;
        this.rgbValues = rgbValues;
    }

    public double getServoValue() {
        return servoValue;
    }

    public String getHexValue() {
        return hexValue;
    }

    public RgbValues getRgbValues() {
        return rgbValues;
    }

//    @Nullable
//    @Contract(pure = true)
//    public static Color fromState(@NonNull State state) {
//        switch (state) {
//            case PURPLE:
//                return VIOLET;
//            case GREEN:
//                return GREEN;
//            case UNKNOWN:
//                return RED;
//            default:
//                return null;
//        }
//    }

    public static class RgbValues {
        public final double r;
        public final double g;
        public final double b;

        public RgbValues(double r, double g, double b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public double[] getArray() {
            return new double[] {r,g,b};
        }
    }

}
