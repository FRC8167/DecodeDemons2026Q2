package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Juggler extends SubsystemBase {

    private final MotorEx spindexer;

    public static final int PPR = 288;
    public static final int SLOTS = 3;
    public static final int COUNTS_PER_SLOT = PPR / SLOTS;
    int target;
    private boolean slowSpinEnabled = false;
    private double slowSpinPower = 0.0;

    private final PIDFController jugglerPID;

    /* wheat 06 FEB 2026 */
    private int normalizedCount;
    private int revolutions;
    /*
        slot 0 -   0 counts to 95  counts, center = 47.5
        slot 1 -  96 counts to 191 counts, center = 143.5
        slot 2 - 192 counts to 287 counts, center = 239.5

        if Slot 0 centered is set to 0 counts
        slot 0 - 240 counts to 47  counts, center = -0.5
        slot 1 -  48 counts to 143 counts, center = 95.5
        slot 2 - 144 counts to 239 counts, center = 190.5
     */
    private int SLOT0_CENTER = 1;
    private int SLOT1_CENTER = 96;
    private int SLOT2_CENTER = 191;

    private int currentSlot;


    public Juggler(MotorEx motor) {
        this.spindexer = motor;
        spindexer.setRunMode(MotorEx.RunMode.RawPower);
        spindexer.resetEncoder();
        spindexer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        jugglerPID = new PIDFController(.01,0,0,0); // ki, kd, kv);  //was.02
        jugglerPID.setTolerance(2);
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
//        target = spindexer.getCurrentPosition() + direction.sign * COUNTS_PER_SLOT;
//        jugglerPID.setSetPoint(target);

        switch (currentSlot) {
            case 0:
                target = (direction.sign > 0) ? SLOT1_CENTER : SLOT2_CENTER;
                break;
            case 1:
                target = (direction.sign > 0) ? SLOT2_CENTER : SLOT0_CENTER;
                break;
            case 2:
                target = (direction.sign > 0) ? SLOT0_CENTER : SLOT1_CENTER;
                break;
        }
//        jugglerPID.setSetPoint(target * revolutions * PPR);  //DMW 2/9
        jugglerPID.setSetPoint(revolutions * PPR + target);  //DMW 2/9

    }


    public void rotateTwoSlots(Direction direction) {
//        target = spindexer.getCurrentPosition() + direction.sign * COUNTS_PER_SLOT*2;
//        jugglerPID.setSetPoint(target);


        switch (currentSlot) {
            case 0:
                target = (direction.sign > 0) ? SLOT2_CENTER : SLOT1_CENTER;
                break;
            case 1:
                target = (direction.sign > 0) ? SLOT0_CENTER : SLOT2_CENTER;
                break;
            case 2:
                target = (direction.sign > 0) ? SLOT1_CENTER : SLOT0_CENTER;
                break;
        }
//        jugglerPID.setSetPoint(target * revolutions * PPR);  //DMW 2/9
        jugglerPID.setSetPoint(revolutions * PPR + target);  //DMW 2/9

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
//        slowSpinEnabled = false;
//        int currentPos = spindexer.getCurrentPosition();
//        int nearestSlot = Math.round((float) currentPos / COUNTS_PER_SLOT);
//        int snappedTarget = nearestSlot * COUNTS_PER_SLOT;
//        jugglerPID.setSetPoint(snappedTarget);
//        target = snappedTarget;

        slowSpinEnabled = false;

        switch (currentSlot) {
            case 0:
                target = SLOT0_CENTER;
                break;
            case 1:
                target = SLOT1_CENTER;
                break;
            case 2:
                target = SLOT2_CENTER;
                break;
        }
//        jugglerPID.setSetPoint(target * revolutions * PPR);  //DMW 2/9
        jugglerPID.setSetPoint(revolutions * PPR + target);  //DMW 2/9

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

        //DMW 2/9 trying to fix normalization
        int currentPosition = spindexer.getCurrentPosition();


        revolutions = Math.floorDiv(currentPosition, PPR);
        normalizedCount = Math.floorMod(currentPosition, PPR);


        /* Added for Debugging Slot Center Error */

//        normalizedCount = currentPosition;
//        if(normalizedCount > 287) {
//            normalizedCount -= 288;
//            revolutions += 1;
//        }
//        else if(normalizedCount < 0)
//            { normalizedCount += 288; revolutions -= 1;
//            }
        updateCurrentSlot();


        //Slow spin = no PID
            if (slowSpinEnabled) {
                //No PID
                spindexer.set(slowSpinPower);
                return;
            }

            // Indexing = w/PID
//            int currentPosition = spindexer.getCurrentPosition();  //DMW 2/9
            double output = Range.clip(
                    jugglerPID.calculate(currentPosition),
                    -0.3, 0.3
            );
            spindexer.set(output);





            updateCurrentSlot();
    }


    public int getNormalizedCount() { return normalizedCount; }


    /**
     * Determines the current slot
     */

//    public void updateCurrentSlot() {
//        if (normalizedCount >= 240 || normalizedCount < 48) {
//            currentSlot = 0;
//        } else if (normalizedCount < 144) {
//            currentSlot = 1;
//        } else {
//            currentSlot = 2;
//        }
//    }

//DMW 2/9  Return to Dave's normalization
    public void updateCurrentSlot() {

        if(normalizedCount >= 240 || normalizedCount < 47) {
            currentSlot = 0;
        } else if(normalizedCount >= 48 && normalizedCount < 143) {
            currentSlot = 1;
        } else if (normalizedCount > 144 && normalizedCount < 239) {
            currentSlot = 2;
        }
    }


}
