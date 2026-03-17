package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Cogintilities.State;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;

public class RotateToStateCommand extends CommandBase {

    private final JugglerAbsolute juggler;
    private final State state;
    private long settleStart = -1;
    private static final long SETTLE_TIME = 250; //ms

    public RotateToStateCommand(JugglerAbsolute juggler, State state) {
        this.juggler = juggler;
        this.state = state;
        addRequirements(juggler);
    }

    @Override
    public void initialize() {
        juggler.rotateToState(state);
    }

    @Override
    public boolean isFinished() {  //DMW 02-13
        if (!juggler.atTarget()) {
            settleStart = -1;
            return false;
        }
        if (settleStart < 0) {
            settleStart = System.currentTimeMillis();
            return false;
        }
        return(System.currentTimeMillis() - settleStart) >= SETTLE_TIME; //TODO: Replace with velocity based settle check

        //return juggler.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor when done
        juggler.stop();
    }
}
