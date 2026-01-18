package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RGBLight extends SubsystemBase {
    private final com.seattlesolvers.solverslib.hardware.servos.ServoEx rgbServo;

    //  From GoBilda documentation:  https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    //  ?srsltid=AfmBOorszs1pIzJrXAfdvvBgkZv74EqZFmsV0VvsG6dhlL0ph21glPFM

    public static final double OFF_POS   = 0.000;
    public static final double RED_POS   = 0.279;
    public static final double ORANGE_POS= 0.333;
    public static final double YELLOW_POS= 0.388;
    public static final double SAGE_POS  = 0.444;
    public static final double GREEN_POS = 0.500;
    public static final double AZURE_POS = 0.555;
    public static final double BLUE_POS  = 0.611;
    public static final double INDIGO_POS= 0.666;
    public static final double VIOLET_POS= 0.722;
    public static final double PINK_POS = 0.75;
    public static final double WHITE_POS = 1.000;

    public enum LightColor {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        PINK,
        WHITE
    }

    private LightColor currentColor = LightColor.OFF;  //store servo position

    //constructor
    public RGBLight(ServoEx rgbServo) {
        this.rgbServo = rgbServo;
        setColor(LightColor.OFF); // default
    }

    //set cases
    private double colorToPosition(LightColor color) {
        switch (color) {
            case RED:     return RED_POS;
            case ORANGE:  return ORANGE_POS;
            case YELLOW:  return YELLOW_POS;
            case SAGE:    return SAGE_POS;
            case GREEN:   return GREEN_POS;
            case AZURE:   return AZURE_POS;
            case BLUE:    return BLUE_POS;
            case INDIGO:  return INDIGO_POS;
            case VIOLET:  return VIOLET_POS;
            case PINK:    return PINK_POS;
            case WHITE:   return WHITE_POS;
            case OFF:
            default:      return OFF_POS;
        }
    }

    //Set solid color
    public void setColor(LightColor color) {
        this.currentColor = color;
        rgbServo.set(colorToPosition(color));
    }

    //Turn off light is needed somewhere
    public void off() {
        setColor(LightColor.OFF);
    }

    //Getter for current color
    public LightColor getCurrentColor() {
        return currentColor;
    }





}
