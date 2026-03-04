package org.firstinspires.ftc.teamcode.Commands;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;

public class SlowSpinPlusInterruptCommand extends CommandBase {

    private final JugglerAbsolute juggler;
    private final JugglerAbsolute.Direction direction;

    public SlowSpinPlusInterruptCommand(JugglerAbsolute juggler, JugglerAbsolute.Direction direction) {
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
