package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class JugglerAbsolute extends SubsystemBase {

    private MotorEx motor;
    private double encoderCount;
    private static double MAX_POWER = 0.5;
    private static double SLOW_SPIN_POWER = 0.2;

    private PIDFController jugglerPID;
    private static int pidTolerance = 3;
    private static double kp, kd, kf;

    /** Pulses per full 360° revolution (encoder resolution). */
    private static final double PULSES_PER_REV = 1425.1;

    /** Slot center positions, indexed 0‑2. */
    private static final double[] SLOT_POSITIONS = {0.0, 95.0, 191.0};

    /* --------------------------------------------------------------------
    1️⃣ Class‑level variables that remember the outcome of the last
        nearest‑slot query.
    -------------------------------------------------------------------- */
    private static int    slotIndex   = -1;   // -1 means “no query yet”
    private static double deltaCounts = 0.0;  // signed error for that slot

    private double slowSpinPower = 0.3;

    enum Mode {POSITION, SLOW_SPIN};
    Mode jugglerMode = Mode.POSITION;

    public enum Direction {
        CW(1),
        CCW(-1);

        public final int sign;

        Direction(int sign) {
            this.sign = sign;
        }
    }



    public JugglerAbsolute(MotorEx jugglerMotor) {
        motor = jugglerMotor;
        motor.setRunMode(MotorEx.RunMode.RawPower);
        motor.resetEncoder();
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        kp = 0.001;
        kd = 0;
        kf = 0;
        jugglerPID = new PIDFController(kp, 0, kd, kf);
        jugglerPID.setTolerance(pidTolerance);
    }


    /* --------------------------------------------------------------------
   2️⃣  Core helper – normalise any raw encoder reading to [0, PPR).
   -------------------------------------------------------------------- */
    private double normalize(double raw) {
        double norm = raw % PULSES_PER_REV;
        if (norm < 0) {
            norm += PULSES_PER_REV;
        }
        return norm;
    }


    /** Finds the slot whose centre is closest to the given normalised position. */
    private int nearestSlotIndex(double normPos) {
        int bestIdx = -1;
        double bestDist = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_POSITIONS.length; i++) {
            double diff = Math.abs(normPos - SLOT_POSITIONS[i]);
            double modDist = diff > PULSES_PER_REV / 2 ? PULSES_PER_REV - diff : diff;
            if (modDist < bestDist) {
                bestDist = modDist;
                bestIdx = i;
            }
        }
        return bestIdx;
    }


    /**
     * Updates the static result fields with the nearest slot information.
     */
    public void updatePosData() {
        encoderCount = motor.getCurrentPosition();
        double normPos = normalize(encoderCount);
        int nearestIdx = nearestSlotIndex(normPos);

        // signed error = currentPosition - slotCenter
        double signedError = normPos - SLOT_POSITIONS[nearestIdx];
        if (Math.abs(signedError) > PULSES_PER_REV / 2) {
            signedError = (signedError > 0)
                    ? signedError - PULSES_PER_REV
                    : signedError + PULSES_PER_REV;
        }

        // Store results in the static fields
        slotIndex   = nearestIdx;
        deltaCounts = signedError;
    }


    /* --------------------------------------------------------------------
       3️⃣  Motion‑generation helpers (use the static result fields when needed)
       -------------------------------------------------------------------- */

    /**
     * Returns the signed error required to move from the current encoder count
     * to the *exact* center of the requested slot.
     *
     * @param targetSlot   slot index (0, 1 or 2)
     * @return signed error (pulses). Positive → rotate forward,
     *         Negative → rotate backward.
     */
//    private double rotate(int targetSlot) {
    private void rotate(int targetSlot) {
        if (targetSlot < 0 || targetSlot >= SLOT_POSITIONS.length) {
            throw new IllegalArgumentException(
                    "targetSlot must be 0, 1, or 2 – received " + targetSlot);
        }

        updatePosData();
        double normPos = normalize(encoderCount);
        double error = normPos - SLOT_POSITIONS[targetSlot];

        if (Math.abs(error) > PULSES_PER_REV / 2) {
            error = (error > 0) ? error - PULSES_PER_REV : error + PULSES_PER_REV;
        }

        jugglerPID.setSetPoint(encoderCount - error);
        jugglerMode = Mode.POSITION;
//        return error;
    }


    /**
     * Moves the disc by a relative number of slots.
     *
     * @param numSlots     number of slots to move (positive = forward,
     *                     negative = backward). Larger magnitudes wrap automatically.
     * @return             signed error (pulses) that will place the disc on the target slot.
     */
    public void moveBySlots(int numSlots) {
        // Populate the static result fields first
        updatePosData();
        int curIdx = getSlotIndex();

        // Compute target index with proper wrap‑around
        int targetIdx = Math.floorMod(curIdx + numSlots, SLOT_POSITIONS.length);
        rotate(targetIdx);
    }


    /** Shortcut: move exactly one slot in the direction specified. */
    public void rotateOneSlot(Juggler.Direction direction) {
        moveBySlots(direction.sign);
    }


    /** Shortcut: move exactly one slot in the direction specified. */
    public void rotateTwoSlots(Juggler.Direction direction) {
        moveBySlots(2 * direction.sign);
    }


    public void snapToNearestSlot() {
        // TODO Add code here
    }


    public void startSlowSpin(Juggler.Direction direction) {
        jugglerMode = Mode.SLOW_SPIN;
        motor.set(SLOW_SPIN_POWER * direction.sign);
    }



    /** @return index (0‑2) of the slot found by the most recent {@code nearestSlot} call */
    public int getSlotIndex() {
        return slotIndex;
    }


    /** @return signed error (pulses) from the most recent {@code nearestSlot} call */
    public double getDeltaCounts() {
        return deltaCounts;
    }


    @Override
    public void periodic() {
        jugglerPID.setPIDF(kp, 0, kd, kf);
        jugglerPID.setTolerance(pidTolerance);

        switch(jugglerMode) {
            case POSITION:
                // TODO Add PID update code
                break;
            case SLOW_SPIN:
                // Do something here? motor is set in motion in start slow spin
                break;
        }
    }

}

// double currentCount = encoder.getCount();   // your encoder reading
// DiscPositioner.nearestSlot(currentCount);   // updates static fields
// int    slotIdx      = DiscPositioner.getLastSlotIndex();
// double errorToSlot  = DiscPositioner.getLastErrorPulses();

// // Example: move to the next slot
// double cmdError = DiscPositioner.nextSlotError(currentCount);
// feed `cmdError` into your PID / motor controller
