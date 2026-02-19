package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LimitSwitch extends SubsystemBase {

    private final DigitalChannel jugglerLimitSwitch;
    private boolean lastState = false;
    private boolean currentState = false;

    public LimitSwitch(DigitalChannel jugglerLimitSwitch) {
        this.jugglerLimitSwitch = jugglerLimitSwitch;
        // Ensure the channel is set to input mode
        this.jugglerLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }
    @Override
    public void periodic() {
        lastState = currentState;
        // Read once per loop to ensure consistency across the subsystem
        currentState = isHome();
    }
    public boolean isHome() {
        // REV magnetic switch is ACTIVE LOW
        return !jugglerLimitSwitch.getState();
    }
    public boolean isRisingEdge() {
        return currentState && !lastState;
    }

    /**
     * Returns true ONLY on the exact loop the magnet leaves the sensor range.
     */
    public boolean isFallingEdge() {
        return !currentState && lastState;
    }

}