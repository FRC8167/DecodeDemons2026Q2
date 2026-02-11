package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Juggler extends SubsystemBase {

    private final MotorEx spindexer;
    private final LimitSwitch limitSwitch;

    private boolean homed = false;
    private boolean hasTarget = false;
    private boolean lastHomeState = false;

    public static final int PPR = 288;
    public static final int SLOTS = 3;
    public static final int COUNTS_PER_SLOT = PPR / SLOTS;

    private int currentSlot = 0;
    private int targetPosition = 0;

    private boolean slowSpinEnabled = false;
    private double slowSpinPower = 0.0;

    private final PIDFController jugglerPID;

    public enum Direction {
        CW(1),
        CCW(-1);

        public final int sign;
        Direction(int sign) {
            this.sign = sign;
        }
    }

    public Juggler(MotorEx motor, LimitSwitch limitSwitch) {
        this.spindexer = motor;
        this.limitSwitch = limitSwitch;

        spindexer.setRunMode(MotorEx.RunMode.RawPower);
        spindexer.resetEncoder();
        spindexer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        jugglerPID = new PIDFController(0.01, 0, 0, 0);
        jugglerPID.setTolerance(3);


    }

    //Return to "HOME"

    public void homeSlow() {

        if (homed) {
            spindexer.stopMotor();
            return;
        }

        spindexer.set(0.15);

        if (limitSwitch.isHome()) {
            spindexer.stopMotor();
            spindexer.resetEncoder();
            jugglerPID.reset();

            homed = true;
            currentSlot = 0;
        }
    }

    // You spin me round (like a record) . . .

    public void startSlowSpin(Direction direction) {
        slowSpinEnabled = true;
        slowSpinPower = 0.2 * direction.sign;
    }

    public void rotateOneSlot(Direction direction) {
        if (!homed) return;

        currentSlot = (currentSlot + direction.sign + SLOTS) % SLOTS;
        jugglerPID.setSetPoint(currentSlot);
    }

    public void rotateTwoSlots(Direction direction) {
        if (!homed) return;
        currentSlot = (currentSlot + 2 * direction.sign + SLOTS) % SLOTS;
        jugglerPID.setSetPoint(currentSlot);
    }

    public void rotateToSlot(int slotIndex) {
        if (!homed) return;

        currentSlot = ((slotIndex % SLOTS) + SLOTS) % SLOTS;
        moveToSlot(currentSlot);
    }

    private void moveToSlot(int slot) {
        targetPosition = slot * COUNTS_PER_SLOT;
        jugglerPID.setSetPoint(targetPosition);
        hasTarget = true;
    }

    // snap to a slot

    public void snapToNearestSlot() {
        if (!homed) return;

        int position = spindexer.getCurrentPosition();

        int nearestSlot = Math.round(position / (float) COUNTS_PER_SLOT);
        nearestSlot = ((nearestSlot % SLOTS) + SLOTS) % SLOTS;

        currentSlot = nearestSlot;
        moveToSlot(currentSlot);
    }

    //Other statusy things

    public boolean atTarget() {
        return jugglerPID.atSetPoint();
    }

    public int getSlotIndex() {
        return currentSlot;
    }

    public int getCurrentPosition() {
        return spindexer.getCurrentPosition();
    }

public void stop(){
        spindexer.stopMotor();
}

    @Override
    public void periodic() {

        //AUTO RE-ZERO EVERY TIME WE PASS HOME????   GOOD, BAD, OR UGLY?
        //NECESSARY FOR ABSOLUTE ENCODERS????
        boolean homeNow = limitSwitch.isHome();

        if (homeNow && !lastHomeState) {
            spindexer.resetEncoder();
            jugglerPID.reset();
            currentSlot = 0;
        }

        lastHomeState = homeNow;

        if (!homed) return;

        if (hasTarget) {
            int currentPosition = spindexer.getCurrentPosition();

            double output = Range.clip(
                    jugglerPID.calculate(currentPosition),
                    -0.35, 0.35
            );

            spindexer.set(output);

            if (jugglerPID.atSetPoint()) {
                hasTarget = false;
            }

        } else {
            spindexer.stopMotor();
        }
    }
}
