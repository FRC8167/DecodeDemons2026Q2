package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Juggler extends SubsystemBase {

    private final MotorEx spindexer;

    public static final int PPR = 288;
    public static final int SLOTS = 3;
    public static final int COUNTS_PER_SLOT = PPR / SLOTS - 1;
    int target;
    private boolean slowSpinEnabled = false;
    private double slowSpinPower = 0.0;

    private final PIDFController jugglerPID;

    private int normalizedCount;
    /*
        slot 0 -   0 counts to 95  counts, center = 47.5
        slot 1 -  96 counts to 191 counts, center = 143.5
        slot 2 - 192 counts to 287 counts, center = 239.5

        if Slot 0 centered is set to 0 counts
        slot 0 - 240 counts to 47  counts, center = -0.5
        slot 1 -  48 counts to 143 counts, center = 95.5
        slot 2 - 144 counts to 239 counts, center = 190.5
     */


    public Juggler(MotorEx motor) {
        this.spindexer = motor;
        spindexer.setRunMode(MotorEx.RunMode.RawPower);
        spindexer.resetEncoder();
        spindexer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        jugglerPID = new PIDFController(.02,0,0,0); // ki, kd, kv);  //was.01
        jugglerPID.setTolerance(5);
        target = 0;
    }


    public enum Direction {
        CW(1),
        CCW(-1);
        public int sign;
        Direction(int sign) {
            this.sign = sign;
        }
    }


    public void rotateOneSlot(Direction direction) {
        target = spindexer.getCurrentPosition() + direction.sign * COUNTS_PER_SLOT;
        jugglerPID.setSetPoint(target);
    }


    public void rotateTwoSlots(Direction direction) {
        target = spindexer.getCurrentPosition() + direction.sign * COUNTS_PER_SLOT*2;
        jugglerPID.setSetPoint(target);
    }


    public void rotateToSlot(int slotIndex) {
        if (slotIndex == 1) { rotateOneSlot(Direction.CW);}
        else if (slotIndex ==2) {rotateOneSlot(Direction.CCW);}
    }


    public void startSlowSpin(Direction direction) {
        slowSpinEnabled = true;
        slowSpinPower = 0.2 * direction.sign;
    }


    public void Snap() {
        slowSpinEnabled = false;
        int currentPos = spindexer.getCurrentPosition();
        int nearestSlot = Math.round((float) currentPos / COUNTS_PER_SLOT);
        int snappedTarget = nearestSlot * COUNTS_PER_SLOT;
        jugglerPID.setSetPoint(snappedTarget);
        target = snappedTarget;
    }

    public void stop(){
        spindexer.set(0.0);
    }


    public boolean atTarget() {
        return jugglerPID.atSetPoint();
    }


    public int getSlotIndex() {
        // unrestricted raw slot number
        int rawSlot = Math.round(spindexer.getCurrentPosition() / (float) COUNTS_PER_SLOT);
        // Restrict slots to 0, 1, or 2
        int slotIndex = ((rawSlot % SLOTS) + SLOTS) % SLOTS;
        return slotIndex;
    }


    @Override
    public void periodic() {
            //Slow spin = no PID
            if (slowSpinEnabled) {
                //No PID
                spindexer.set(slowSpinPower);
                return;
            }

            // Indexing = w/PID
            int currentPosition = spindexer.getCurrentPosition();
            double output = Range.clip(
                    jugglerPID.calculate(currentPosition),
                    -0.3, 0.3
            );
            spindexer.set(output);

            /* Added for Debugging Slot Center Error */
            normalizedCount = (int)currentPosition;
            if(normalizedCount > 287) normalizedCount = (int)currentPosition - 288;
    }


    public int getRelativeCount() { return normalizedCount; }


    /**
     * Calculates the difference between the actual Slot center position and the theoretical
     * @return Difference in counts. Positive errors indicate the juggler rotated more CW than theoretical
     */
    public double getSlotCenterError() {
        double error = -999;

        if(normalizedCount >= 240 || normalizedCount < 47) {
            error = normalizedCount - 0.5;
        } else if(normalizedCount >= 48 && normalizedCount < 143) {
            error = normalizedCount - 95.5;
        } else if (normalizedCount > 144 && normalizedCount < 239) {
            error = normalizedCount - 190.5;
        }

        return error;
    }

}
