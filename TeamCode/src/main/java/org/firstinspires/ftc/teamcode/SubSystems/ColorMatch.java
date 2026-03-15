package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.EnumMap;

public class ColorMatch extends SubsystemBase {

    private final EnumMap<Slot, RevColorSensorV3> sensors = new EnumMap<>(Slot.class);
    // Purple is usually around 270-300 hue
    private final double PURPLE_HUE_MIN = 185;
    private final double PURPLE_HUE_MAX = 320;

    // Green is usually around 100-140 hue
    private final double GREEN_HUE_MIN = 90;
    private final double GREEN_HUE_MAX = 170;

    public ColorMatch(
            @NonNull RevColorSensorV3 slot0Sensor,
            @NonNull RevColorSensorV3 slot1Sensor,
            @NonNull RevColorSensorV3 slot2Sensor
    ) {
        sensors.put(Slot.SLOT_0, slot0Sensor);
        sensors.put(Slot.SLOT_1, slot1Sensor);
        sensors.put(Slot.SLOT_2, slot2Sensor);
    }


    public enum Slot {
        SLOT_0,
        SLOT_1,
        SLOT_2
    }


    public enum ArtifactColor {
        NONE,
        GREEN,
        PURPLE,
        UNKNOWN
    }



    public float[] getHSV(Slot slot) {

        final float[] hsvValues = new float[3];
        // Define the sensor
        RevColorSensorV3 sensor = sensors.get(slot);
        if (sensor == null) return new float[]{0, 0, 0};

        sensor.setGain(2.0f);
        //turn off/on led
        sensor.enableLed(false);

        NormalizedRGBA colors = sensor.getNormalizedColors();
        android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);


        return hsvValues;
    }

    //Trying to distinguish empty from unknown

    public ArtifactColor detectColor(Slot slot) {
        RevColorSensorV3 sensor = sensors.get(slot);
        if (sensor == null) return null;

        float[] hsv = getHSV(slot);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];
        float distance = (float) sensor.getDistance(DistanceUnit.CM);
        //if (val < 0.15 || sat < 0.35) {return ArtifactColor.UNKNOWN;}
        // Distance threshold (in CM)
        double DETECTION_DISTANCE_CM = getDetectionDistanceCm(slot);
        if (distance < DETECTION_DISTANCE_CM) {
            if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                return ArtifactColor.GREEN;
            } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                return ArtifactColor.PURPLE;
            } else {
                return ArtifactColor.UNKNOWN;
            }
        }
        else {
            return ArtifactColor.NONE;
        }



    }

    private static double getDetectionDistanceCm(Slot slot) {
        double DETECTION_DISTANCE_CM = 5.05;

        // NOTE: Review fall-through logic here. Currently, all
        // values are overwritten by default case because 'break' is missing.

        switch (slot) {

            case SLOT_0:
                DETECTION_DISTANCE_CM = 5.7;
            case SLOT_1:
                DETECTION_DISTANCE_CM = 5.8;
            case SLOT_2:
                DETECTION_DISTANCE_CM = 9;
            default:
                DETECTION_DISTANCE_CM = 5.5;
        }
        return DETECTION_DISTANCE_CM;
    }


//    public ArtifactColor detectColor(Slot slot) {
//        float[] hsv = getHSV(slot);
//        float hue = hsv[0];
//        float sat = hsv[1];
//        float val = hsv[2];
//        //if (val < 0.15 || sat < 0.35) {return ArtifactColor.UNKNOWN;}
//
//        if ((sat == 0 && hue == 0) || (sat == 1 && (hue == 120 || hue == 60))) // air is 120 and value !=0 for some reason
//        {return ArtifactColor.UNKNOWN;}
//
    ////        if (sat < 0.2 || val < 0.2) return ArtifactColor.UNKNOWN;
//        if (hue > 70 && hue < 160 && hue !=120) return ArtifactColor.GREEN;
//        if (hue > 220 && hue < 350) return ArtifactColor.PURPLE;
//
//        return ArtifactColor.UNKNOWN;
//    }


    public static class SlotColors {
        public final ArtifactColor slot0;
        public final ArtifactColor slot1;
        public final ArtifactColor slot2;

        public SlotColors(ArtifactColor slot0,
                          ArtifactColor slot1,
                          ArtifactColor slot2) {
            this.slot0 = slot0;
            this.slot1 = slot1;
            this.slot2 = slot2;
        }
    }


    public SlotColors getSlotColors() {
        return new SlotColors(
                detectColor(Slot.SLOT_0),
                detectColor(Slot.SLOT_1),
                detectColor(Slot.SLOT_2)
        );
    }


    public int findSlotWithColor(ArtifactColor targetColor) {
        SlotColors slots = getSlotColors();
        if (slots.slot0 == targetColor) return 0;
        if (slots.slot1 == targetColor) return 1;
        if (slots.slot2 == targetColor) return 2;
        return -1; // not found
    }

    public double getDistance(Slot slot){
        RevColorSensorV3 sensor = sensors.get(slot);
        if (sensor == null) return Double.NaN;
        return sensor.getDistance(DistanceUnit.CM);
    }

    public void updateSpinStates(int jugglerIndex) {
        SpinStatesSingleton_Eric.getInstance().updateByColorMatchColors(getSlotColors(), jugglerIndex);
    }

    public void forceUpdateSpinStates(int jugglerIndex) {
        SpinStatesSingleton_Eric.getInstance().forceSetByColorMatchColors(getSlotColors(), jugglerIndex);
    }
}