package org.firstinspires.ftc.teamcode.Commands;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;

public class SlowSpinPlusInterruptCommand extends CommandBase {

    private final JugglerAbsolute_EXP juggler;
    private final JugglerAbsolute_EXP.Direction direction;

    public SlowSpinPlusInterruptCommand(JugglerAbsolute_EXP juggler, JugglerAbsolute_EXP.Direction direction) {
        this.juggler = juggler;
        this.direction = direction;
        addRequirements(juggler);
    }

    @Override
    public void initialize() {
        juggler.startSlowSpin(direction);
    }

    @Override
    public boolean isFinished() {
        return false; // deadline controls lifetime
    }

    @Override
    public void end(boolean interrupted) {
        juggler.snapToNearestSlot();
    }
}
