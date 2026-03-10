package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BetterMotor extends DcMotorImplEx {

    public BetterMotor(@NonNull DcMotorEx motor) {
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
    }

    public BetterMotor(@NonNull HardwareMap hardwareMap, String motorName) {
        this(hardwareMap.get(DcMotorEx.class, motorName));
    }


    public void setVelocityRPM(double velocityRPM) {
        setVelocity(RPMtoDPS(velocityRPM), AngleUnit.DEGREES);
    }

    public double getVelocityRPM() {
        return DPStoRPM(getVelocity(AngleUnit.DEGREES));
    }

    public void setTargetPosition(double position, @NonNull AngleUnit unit) {
        int ticks = degToTicks(unit.getUnnormalized().toDegrees(position));
        super.setTargetPosition(ticks);
    }

    public double getTargetPosition(@NonNull AngleUnit unit) {
        int ticks = super.getTargetPosition();
        return unit.getUnnormalized().fromDegrees(ticksToDeg(ticks));
    }

    public double getCurrentPosition(@NonNull AngleUnit unit) {
        int ticks = super.getCurrentPosition();
        return unit.getUnnormalized().fromDegrees(ticksToDeg(ticks));
    }

    public void adjustMotorInformation(MotorInformation motorInfo) {
        MotorInformation.adjustMotor(this, motorInfo);
    }





    public double ticksToDeg(double ticks) {
        return ticks/getTicksPerDeg();
    }

    public int degToTicks(double degrees) {
        return (int) (degrees*getTicksPerDeg());
    }

    public double getTicksPerRev() {
        return getMotorType().getTicksPerRev();
    }

    public double getTicksPerDeg() {
        return getTicksPerRev() / 360.0;
    }

    public double DPStoRPM(double dps) {
        return dps/6.0;
    }

    public double RPMtoDPS(double rpm) {
        return rpm*6.0;
    }


}
