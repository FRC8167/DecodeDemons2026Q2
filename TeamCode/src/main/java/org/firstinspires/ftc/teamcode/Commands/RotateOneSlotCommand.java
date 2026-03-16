package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;

public class RotateOneSlotCommand extends CommandBase {

    private final JugglerAbsolute_EXP juggler;
    private final JugglerAbsolute_EXP.Direction direction;

    public RotateOneSlotCommand(JugglerAbsolute_EXP juggler, JugglerAbsolute_EXP.Direction direction) {
        this.juggler = juggler;
        this.direction = direction;
        addRequirements(juggler);
    }

    @Override
    public void initialize() {
        juggler.rotateOneSlot(direction);
    }

    @Override
    public boolean isFinished() {
        return juggler.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor when done
//        juggler.stop();
    }
}
