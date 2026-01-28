package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Popper extends SubsystemBase {

    public enum PopperState {
        KICK,
        RESET
    }

    private final ServoEx popperServoL, popperServoR;
    private PopperState currentState = PopperState.RESET; // tracks current state

    public Popper(ServoEx popperServoL, ServoEx popperServoR) {
        this.popperServoL = popperServoL;
        this.popperServoR = popperServoR;
        this.popperServoR.setInverted(true);
        this.popperServoL.set(0.47);
        this.popperServoR.set(0.49);
    }

    public void set(PopperState state) {
        currentState = state;
        switch (state) {
            case KICK:
                popperServoL.set(0.75);//.25
                popperServoR.set(.75);
                break;
            case RESET:
                popperServoL.set(0.47);//
                popperServoR.set(.49);//.47
                break;
        }
    }

    public void midServo() {
    popperServoL.set(0.5);
    popperServoR.set(0.5);
    }

    public PopperState getPopperState() {
        return currentState;
    }
}
