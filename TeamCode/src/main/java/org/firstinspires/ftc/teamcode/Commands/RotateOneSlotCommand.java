package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;

public class RotateOneSlotCommand extends CommandBase {

    private final Juggler juggler;
    private final Juggler.Direction direction;

    public RotateOneSlotCommand(Juggler juggler, Juggler.Direction direction) {
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
