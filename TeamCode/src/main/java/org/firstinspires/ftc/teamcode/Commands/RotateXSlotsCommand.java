package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;

public class RotateXSlotsCommand extends CommandBase {

    private final Juggler juggler;
    private final Juggler.Direction direction;
    private final int num;

    public RotateXSlotsCommand(Juggler juggler, Juggler.Direction direction, int num) {
        this.juggler = juggler;
        this.direction = direction;
        this.num = num;
        addRequirements(juggler);
    }

    @Override
    public void initialize() {
        if (num == 0){ return;}
        else if (num == 1) {
            juggler.rotateOneSlot(direction);
            return;
        }
        else juggler.rotateTwoSlots(direction);
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
