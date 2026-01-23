package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Commands.RotateOneSlotCommand;

public class Juggler extends SubsystemBase {

    private final MotorEx spindexer;

    public static final int PPR = 288;
    public static final int SLOTS = 3;
    public static final int COUNTS_PER_SLOT = PPR / SLOTS;
    int target;
    private boolean slowSpinEnabled = false;
    private double slowSpinPower = 0.0;

    private final PIDFController jugglerPID;

    public Juggler(MotorEx motor) {
        this.spindexer = motor;
        spindexer.setRunMode(MotorEx.RunMode.RawPower);
        spindexer.resetEncoder();
        spindexer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        jugglerPID = new PIDFController(.02,0,0,0); // ki, kd, kv);  //was.01
        jugglerPID.setTolerance(10);
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
        slowSpinPower = 0.38 * direction.sign;
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
            double currentPosition = spindexer.getCurrentPosition();
            double output = Range.clip(
                    jugglerPID.calculate(currentPosition),
                    -0.3, 0.3
            );
            spindexer.set(output);
        }





}
