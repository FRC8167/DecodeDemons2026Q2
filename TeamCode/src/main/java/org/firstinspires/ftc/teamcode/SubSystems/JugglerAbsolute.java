package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.BetterMotor;
import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.DefaultMotorInfo;

@Configurable
public class JugglerAbsolute extends SubsystemBase {

    private BetterMotor motor;
    private static double SLOW_SPIN_RPM = 20;

    private static int PID_TOLERANCE = 8;   // 8/1425.1*360 = 2 degrees
    private static double kp, ki, kd, kf, kp_pos, maxRPM;

    /** Pulses per full revolution (encoder resolution). */
//     private static final double PULSES_PER_REV = 288;
//    private static final double PULSES_PER_REV = 1425.1;

    /** Pre‑defined slot centres (pulse counts). */
//     private static final double[] SLOT_CENTRES = {0.0, 96.0, 192.0};
    private static final double[] SLOT_CENTERS = {0.0, 475.0, 950.0};

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
    public JugglerAbsolute(BetterMotor jugglerMotor) {
        motor = jugglerMotor;
        motor.adjustMotorInformation(DefaultMotorInfo.GOBILDA_117RPM);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kp = 10.0;
        ki = 0.25;
        kd = 0.0;
        kf = 14.0;
        kp_pos = 8;

        maxRPM = 35;

        motor.setVelocityPIDFCoefficients(kp, ki, kd, kf);
        motor.setPositionPIDFCoefficients(kp_pos);
        motor.setTargetPositionTolerance(PID_TOLERANCE);
    }

    /* --------------------------------------------------------------
     * 1️⃣  Legacy call wrappers
     * -------------------------------------------------------------- */
    public void rotateOneSlot(Direction direction) {
        boolean dir = direction.sign == 1;
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
        motor.setTargetPosition((int) counts);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocityRPM(maxRPM);

    }

    /**
     *
     * @param direction
     */
    public void startSlowSpin(Direction direction) {
        jugglerMode = Mode.SLOW_SPIN;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double slowSpinRPM = (direction.sign==1) ? SLOW_SPIN_RPM : -SLOW_SPIN_RPM;
        motor.setVelocityRPM(slowSpinRPM);motor.isBusy();
    }

    /**
     *
     */
    public void stop() {
        motor.setPower(0);
    }

    /**
     *
     * @return
     */
    public boolean atTarget() {
        double error = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        return error < PID_TOLERANCE;
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

    }

    /* --------------------------------------------------------------
     * 1️⃣  Core public API
     * -------------------------------------------------------------- */

    public double nearestSlotSetpoint(double currentCount) {
        int nearestIdx = nearestSlotIndex(currentCount);
        return slotSetpoint(currentCount, nearestIdx);
    }

    public int nearestSlotIndex(double currentCount) {
        double normPos = mod(currentCount, motor.getTicksPerRev());
        int bestIdx = -1;
        double bestAbsDelta = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_CENTERS.length; i++) {
            double delta = shortestSignedDelta(normPos, SLOT_CENTERS[i]);
            double abs   = Math.abs(delta);
            if (abs < bestAbsDelta) {
                bestAbsDelta = abs;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    public double slotSetpoint(double currentCount, int slotIndex) {
        if (slotIndex < 0 || slotIndex >= SLOT_CENTERS.length) {
            throw new IllegalArgumentException(
                    "slotIndex must be 0, 1, or 2 (was " + slotIndex + ')');
        }
        double normPos = mod(currentCount, motor.getTicksPerRev());
        double delta   = shortestSignedDelta(normPos, SLOT_CENTERS[slotIndex]);
        return currentCount + delta;
    }

    public double nextSlotSetpoint(double currentCount) {
        return nextSlotSetpoint(currentCount, true);
    }

    public double nextSlotSetpoint(double currentCount, boolean clockwise) {
        int nearestIdx = nearestSlotIndex(currentCount);
        int nextIdx = clockwise
                ? (nearestIdx + 1) % SLOT_CENTERS.length
                : (nearestIdx - 1 + SLOT_CENTERS.length) % SLOT_CENTERS.length;
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
    public double moveToSlot(double currentCount, double slotCenterPulse) {
        // Normalise the target centre to the same 0‑rev range as the current pos.
        double normalisedTarget = mod(slotCenterPulse, motor.getTicksPerRev());
        double normPos          = mod(currentCount, motor.getTicksPerRev());

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
    public double moveToSlotByIndex(double currentCount, int slotIndex) {
        if (slotIndex < 0 || slotIndex >= SLOT_CENTERS.length) {
            throw new IllegalArgumentException(
                    "slotIndex must be 0, 1, or 2 (was " + slotIndex + ')');
        }
        return moveToSlot(currentCount, SLOT_CENTERS[slotIndex]);
    }

    /**
     * Returns the set‑point that moves **N slots** from the *nearest* slot.
     *
     * @param currentCount   Current raw encoder count.
     * @param slotShift      Number of slots to move.
     *                       Positive → clockwise, Negative → counter‑clockwise.
     * @return               Absolute encoder count for the target slot.
     */
    public double rotateBySlots(double currentCount, int slotShift) {
        // 1️⃣ Find the nearest slot index.
        int nearestIdx = nearestSlotIndex(currentCount);

        // 2️⃣ Compute the index of the destination slot.
        //    Use modulo arithmetic to wrap around the three slots.
        int destinationIdx = ((nearestIdx + slotShift) % SLOT_CENTERS.length
                + SLOT_CENTERS.length) % SLOT_CENTERS.length;

        // 3️⃣ Return the set‑point for that destination slot.
        return slotSetpoint(currentCount, destinationIdx);
    }


    /* --------------------------------------------------------------
     * 3️⃣  Private helper utilities (unchanged)
     * -------------------------------------------------------------- */

    /** Normalises a value into the interval [0, modulus). */
    private double mod(double value, double modulus) {
        double result = value % modulus;
        return (result < 0) ? result + modulus : result;
    }

    /**
     * Signed angular distance from {@code from} to {@code to},
     * wrapped to the shortest path (‑½ rev … +½ rev).
     *
     * Positive → clockwise, Negative → counter‑clockwise
     */
    private double shortestSignedDelta(double from, double to) {
        double rawDelta = to - from;
        if (rawDelta >  motor.getTicksPerRev() / 2.0) rawDelta -= motor.getTicksPerRev();
        if (rawDelta < -motor.getTicksPerRev() / 2.0) rawDelta += motor.getTicksPerRev();
        return rawDelta;
    }

}

