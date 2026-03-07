package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;


import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.jetbrains.annotations.Contract;

public final class GearedMotorInfo implements MotorInformation {

    private final MotorInformation baseMotorInfo;
    private final double additionalGearRatio;
    private final boolean reversed;

    public GearedMotorInfo(MotorInformation baseMotorInfo, double additionalGearRatio) {
        this.baseMotorInfo = baseMotorInfo;
        this.additionalGearRatio = additionalGearRatio;
        this.reversed = false;
    }

    public GearedMotorInfo(MotorInformation baseMotorInfo, double additionalGearRatio, boolean reversed) {
        this.baseMotorInfo = baseMotorInfo;
        this.additionalGearRatio = additionalGearRatio;
        this.reversed = reversed;
    }

    public GearedMotorInfo(MotorInformation baseMotorInfo, boolean reversed) {
        this.baseMotorInfo = baseMotorInfo;
        this.additionalGearRatio = 1;
        this.reversed = reversed;
    }

    @Override
    public double getTicksPerRev() {
        return baseMotorInfo.getTicksPerRev() * additionalGearRatio;
    }

    @Override
    public double getMaxRPM() {
        return baseMotorInfo.getMaxRPM() / additionalGearRatio;
    }

    @Override
    public double getGearing() {
        return baseMotorInfo.getGearing() * additionalGearRatio;
    }

    @Override
    public Rotation getOrientation() {
        if (!reversed)
            return baseMotorInfo.getOrientation();
        else
            return reverse(baseMotorInfo.getOrientation());
    }

    @Override
    public boolean isReversed() {
        return reversed;
    }

    @Nullable
    @Contract(pure = true)
    private Rotation reverse(Rotation rotation) {
        if (rotation == Rotation.CW)
            return Rotation.CCW;
        else if (rotation == Rotation.CCW)
            return Rotation.CW;
        else
            return null;
    }

    // No need to implement adjustMotor()
}
