package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;

import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public enum DefaultMotorInfo implements MotorInformation {
    GOBILDA_6000RPM(28,     6000, 1   , Rotation.CCW),
    GOBILDA_1620RPM(103.8,  1620, 3.7 , Rotation.CCW),
    GOBILDA_1150RPM(145.1,  1150, 5.2 , Rotation.CCW),
    GOBILDA_435RPM (384.5,  435,  13.7, Rotation.CCW),
    GOBILDA_312RPM (537.7,  312,  19.2, Rotation.CCW),
    GOBILDA_223RPM (751.8,  223,  26.9, Rotation.CCW),
    GOBILDA_117RPM (1425.1, 117,  50.9, Rotation.CCW),
    GOBILDA_84RPM  (1993.6, 84,   71.2, Rotation.CCW),
    GOBILDA_60RPM  (2786.2, 60,   99.5, Rotation.CCW),
    GOBILDA_43RPM  (3895.9, 43,   139 , Rotation.CCW),
    GOBILDA_30RPM  (5281.1, 30,   188 , Rotation.CCW),

    REV_HEX_125RPM (288,    125,  72,   Rotation.CCW);

    private final double ticksPerRev;
    private final double maxRPM;
    private final double gearing;
    private final Rotation rotation;

    DefaultMotorInfo(double ticksPerRev, double maxRPM, double gearing, Rotation rotation) {
        this.ticksPerRev = ticksPerRev;
        this.maxRPM = maxRPM;
        this.gearing = gearing;
        this.rotation = rotation;
    }

    @Override
    public double getTicksPerRev() {
        return ticksPerRev;
    }

    @Override
    public double getMaxRPM() {
        return maxRPM;
    }

    @Override
    public double getGearing() {
        return gearing;
    }

    @Override
    public Rotation getOrientation() {
        return rotation;
    }


//    @Override
//    public void adjustMotor(@NonNull DcMotor motor) {
//        MotorInformation.adjustMotor(motor, this);
//    }

//    public static void adjustMotor(@NonNull DcMotor motor, @NonNull MotorInformation motorInfo) {
//        MotorConfigurationType newConfig = motor.getMotorType().clone();
//        newConfig.setTicksPerRev(motorInfo.getTicksPerRev());
//        newConfig.setMaxRPM(motorInfo.getMaxRPM());
//        newConfig.setGearing(motorInfo.getGearing());
//        motor.setMotorType(newConfig);
//    }
}
