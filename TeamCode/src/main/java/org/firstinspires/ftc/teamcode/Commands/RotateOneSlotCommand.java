package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;

public class RotateOneSlotCommand extends CommandBase {

    private final JugglerAbsolute juggler;
    private final JugglerAbsolute.Direction direction;

    public RotateOneSlotCommand(JugglerAbsolute juggler, JugglerAbsolute.Direction Direction) {
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
        juggler.stop();
    }
}
