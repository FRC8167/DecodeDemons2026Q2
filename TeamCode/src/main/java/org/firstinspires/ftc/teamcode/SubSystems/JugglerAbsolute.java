package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class JugglerAbsolute extends SubsystemBase {

    private MotorEx motor;
    private double encoderCount;
    private static double MAX_POWER = 0.3;
    private static double SLOW_SPIN_POWER = 0.2;

    private PIDFController jugglerPID;
    private static int pidTolerance = 8;
    private static double kp, ki, kd, kf;

    /** Pulses per full revolution (encoder resolution). */
//     private static final double PULSES_PER_REV = 288;
    private static final double PULSES_PER_REV = 1425.1;

    /** Pre‑defined slot centres (pulse counts). */
//     private static final double[] SLOT_CENTRES = {0.0, 96.0, 192.0};
    private static final double[] SLOT_CENTRES = {0.0, 475.0, 950.0};

    public enum Direction {
        CW(1),
        CCW(-1);

        public final int sign;

        Direction(int sign) {
            this.sign = sign;
        }
    }

    enum Mode {POSITION, SLOW_SPIN};
    Mode jugglerMode = Mode.POSITION;

    /* --------------------------------------------------------------
     * 1️⃣  Constructor
     * -------------------------------------------------------------- */
    public JugglerAbsolute(MotorEx jugglerMotor) {
        motor = jugglerMotor;
        motor.setRunMode(MotorEx.RunMode.RawPower);
        motor.resetEncoder();
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        kp = 0.015;
        kd = 0.0004;
        ki = 0.0;
        kf = 0.0;
        jugglerPID = new PIDFController(kp, ki, kd, kf);
        jugglerPID.setTolerance(pidTolerance);
    }

    /* --------------------------------------------------------------
     * 1️⃣  Legacy call wrappers
     * -------------------------------------------------------------- */
    public void rotateOneSlot(Direction direction) {
        boolean dir = (direction.sign == 1) ? true : false;
        startMotion(nextSlotSetpoint(motor.getCurrentPosition(), dir));
    }

    /**
     *
     * @param direction
     */
    public void rotateTwoSlots(Direction direction) {
        int slots = (direction.sign == 1) ? 2 : -2;
        double newTarget = rotateBySlots(motor.getCurrentPosition(), slots);
        startMotion(newTarget);
    }

    /**
     *
     * @param direction
     */
    public void jogThree(Direction direction) {
        int slots = (direction.sign == 1) ? 3 : -3;
        double newTarget = rotateBySlots(motor.getCurrentPosition(), slots);
        startMotion(newTarget);
    }

    /**
     *
     * @param slotIndex
     */
    public void rotateToSlot(int slotIndex) {
        startMotion(moveToSlotByIndex(motor.getCurrentPosition(), slotIndex));
    }

    /**
     *
     */
    public void snapToNearestSlot() {
        startMotion(nearestSlotSetpoint(motor.getCurrentPosition()));
    }

    /**
     *
     * @param counts
     */
    private void startMotion(double counts) {
        jugglerMode = Mode.POSITION;
        jugglerPID.setSetPoint ((int) counts);
    }

    /**
     *
     * @param direction
     */
    public void startSlowSpin(Direction direction) {
        jugglerMode = Mode.SLOW_SPIN;
        double slowSpinPower = (direction.sign==1) ? SLOW_SPIN_POWER : -SLOW_SPIN_POWER;
        motor.set(slowSpinPower);
    }

    /**
     *
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     *
     * @return
     */
    public boolean atTarget() {
        return jugglerPID.atSetPoint();
    }


    public boolean isReadyToFire() {
        double curr = motor.getCurrentPosition();
        double nearestSlot = nearestSlotSetpoint(curr);

        return Math.abs(curr - nearestSlot) < 12;
    }

    /**
     * Should not need this function once new motor is installed
     */
    public void goHome() {
        startMotion(0);
    }

    /**
     *
     * @return
     */
    public int getSlotIndex() {
        return nearestSlotIndex(motor.getCurrentPosition());
    }


    @Override
    public void periodic() {

        // TODO: Remove next two lines for competition - for tuning purposes only
        jugglerPID.setPIDF(kp, ki, kd, kf);
        jugglerPID.setTolerance(pidTolerance);

        switch(jugglerMode) {
            case POSITION:
                int currentPosition = motor.getCurrentPosition();
                double controlOutput = Range.clip(jugglerPID.calculate(currentPosition), -MAX_POWER, MAX_POWER);
                motor.set(controlOutput);
                // double error = jugglerPID.get
                break;
            case SLOW_SPIN:
                // Do something here? motor is set in motion in start slow spin
                break;
        }
    }

    /* --------------------------------------------------------------
     * 1️⃣  Core public API
     * -------------------------------------------------------------- */

    public static double nearestSlotSetpoint(double currentCount) {
        int nearestIdx = nearestSlotIndex(currentCount);
        return slotSetpoint(currentCount, nearestIdx);
    }

    public static int nearestSlotIndex(double currentCount) {
        double normPos = mod(currentCount, PULSES_PER_REV);
        int bestIdx = -1;
        double bestAbsDelta = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_CENTRES.length; i++) {
            double delta = shortestSignedDelta(normPos, SLOT_CENTRES[i]);
            double abs   = Math.abs(delta);
            if (abs < bestAbsDelta) {
                bestAbsDelta = abs;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    public static double slotSetpoint(double currentCount, int slotIndex) {
        if (slotIndex < 0 || slotIndex >= SLOT_CENTRES.length) {
            throw new IllegalArgumentException(
                    "slotIndex must be 0, 1, or 2 (was " + slotIndex + ')');
        }
        double normPos = mod(currentCount, PULSES_PER_REV);
        double delta   = shortestSignedDelta(normPos, SLOT_CENTRES[slotIndex]);
        return currentCount + delta;
    }

    public static double nextSlotSetpoint(double currentCount) {
        return nextSlotSetpoint(currentCount, true);
    }

    public static double nextSlotSetpoint(double currentCount, boolean clockwise) {
        int nearestIdx = nearestSlotIndex(currentCount);
        int nextIdx = clockwise
                ? (nearestIdx + 1) % SLOT_CENTRES.length
                : (nearestIdx - 1 + SLOT_CENTRES.length) % SLOT_CENTRES.length;
        return slotSetpoint(currentCount, nextIdx);
    }

    /**
     * Moves to an **arbitrary slot center** supplied as a pulse count.
     *
     * @param currentCount     The current raw encoder count (any sign/value).
     * @param slotCenterPulse  The pulse position of the desired slot centre.
     *                         It may be inside [0, PULSES_PER_REV) or outside;
     *                         the method automatically normalises it.
     * @return absolute encoder set‑point that reaches the supplied slot centre
     *         using the shortest angular travel (clockwise or counter‑clockwise).
     */
    public static double moveToSlot(double currentCount, double slotCenterPulse) {
        // Normalise the target centre to the same 0‑rev range as the current pos.
        double normalisedTarget = mod(slotCenterPulse, PULSES_PER_REV);
        double normPos          = mod(currentCount, PULSES_PER_REV);

        double delta = shortestSignedDelta(normPos, normalisedTarget);
        return currentCount + delta;
    }

    /**
     * Convenience wrapper that accepts a slot index (0‑2) and forwards to
     * {@link #moveToSlot(double,double)}. This illustrates how you could
     * expose the same interface for a dynamic list of slots.
     *
     * @param currentCount the present raw encoder count
     * @param slotIndex    0, 1 or 2 – selects one of the three fixed slots
     * @return target encoder count for the requested slot
     */
    public static double moveToSlotByIndex(double currentCount, int slotIndex) {
        if (slotIndex < 0 || slotIndex >= SLOT_CENTRES.length) {
            throw new IllegalArgumentException(
                    "slotIndex must be 0, 1, or 2 (was " + slotIndex + ')');
        }
        return moveToSlot(currentCount, SLOT_CENTRES[slotIndex]);
    }

    /**
     * Returns the set‑point that moves **N slots** from the *nearest* slot.
     *
     * @param currentCount   Current raw encoder count.
     * @param slotShift      Number of slots to move.
     *                       Positive → clockwise, Negative → counter‑clockwise.
     * @return               Absolute encoder count for the target slot.
     */
    public static double rotateBySlots(double currentCount, int slotShift) {
        // 1️⃣ Find the nearest slot index.
        int nearestIdx = nearestSlotIndex(currentCount);

        // 2️⃣ Compute the index of the destination slot.
        //    Use modulo arithmetic to wrap around the three slots.
        int destinationIdx = ((nearestIdx + slotShift) % SLOT_CENTRES.length
                + SLOT_CENTRES.length) % SLOT_CENTRES.length;

        // 3️⃣ Return the set‑point for that destination slot.
        return slotSetpoint(currentCount, destinationIdx);
    }


    /* --------------------------------------------------------------
     * 3️⃣  Private helper utilities (unchanged)
     * -------------------------------------------------------------- */

    /** Normalises a value into the interval [0, modulus). */
    private static double mod(double value, double modulus) {
        double result = value % modulus;
        return (result < 0) ? result + modulus : result;
    }

    /**
     * Signed angular distance from {@code from} to {@code to},
     * wrapped to the shortest path (‑½ rev … +½ rev).
     *
     * Positive → clockwise, Negative → counter‑clockwise
     */
    private static double shortestSignedDelta(double from, double to) {
        double rawDelta = to - from;
        if (rawDelta >  PULSES_PER_REV / 2) rawDelta -= PULSES_PER_REV;
        if (rawDelta < -PULSES_PER_REV / 2) rawDelta += PULSES_PER_REV;
        return rawDelta;
    }

}

