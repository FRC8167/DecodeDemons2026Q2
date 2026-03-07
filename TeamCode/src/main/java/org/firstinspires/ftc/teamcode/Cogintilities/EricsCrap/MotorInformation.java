package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * An interface representing the physical properties of a DC motor.
 * Allows for standard motor types and custom geared motors to be used interchangeably.
 */
public interface MotorInformation {

    double getTicksPerRev();
    double getMaxRPM();
    double getGearing();
    Rotation getOrientation();

    default boolean isReversed() {
        return false;
    }


    /**
     * Applies the motor's properties to a DcMotor instance.
     * This default method can be used by any class that implements the interface.
     */
    default void adjustMotor(@NonNull DcMotor motor) {
        MotorConfigurationType newConfig = motor.getMotorType().clone();
        newConfig.setTicksPerRev(this.getTicksPerRev());
        newConfig.setMaxRPM(this.getMaxRPM());
        newConfig.setGearing(this.getGearing());
        motor.setMotorType(newConfig);
    }

    static void adjustMotor(@NonNull DcMotor motor, @NonNull MotorInformation motorInfo) {
        MotorConfigurationType newConfig = motor.getMotorType().clone();
        newConfig.setTicksPerRev(motorInfo.getTicksPerRev());
        newConfig.setMaxRPM(motorInfo.getMaxRPM());
        newConfig.setGearing(motorInfo.getGearing());
        newConfig.setOrientation(motorInfo.getOrientation());
        motor.setMotorType(newConfig);
    }


}
