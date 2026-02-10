package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LimitSwitch extends SubsystemBase {

    private final DigitalChannel jugglerLimitSwitch;

    public LimitSwitch(DigitalChannel jugglerLimitSwitch) {
        this.jugglerLimitSwitch = jugglerLimitSwitch;
    }

    public boolean isHome() {
        // REV magnetic switch is ACTIVE LOW
        return !jugglerLimitSwitch.getState();
    }


}