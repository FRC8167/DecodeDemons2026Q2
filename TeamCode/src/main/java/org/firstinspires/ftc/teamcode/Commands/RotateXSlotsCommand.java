package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;

public class RotateXSlotsCommand extends CommandBase {

    private final JugglerAbsolute_EXP juggler;
    private final JugglerAbsolute_EXP.Direction direction;
    private final int num;
    private long settleStart = -1;
    private static final long SETTLE_TIME = 250; //ms

    public RotateXSlotsCommand(JugglerAbsolute_EXP juggler, JugglerAbsolute_EXP.Direction direction, int num) {
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
    public boolean isFinished() {  //DMW 02-13
        if (!juggler.atTarget()) {
            settleStart = -1;
            return false;
        }
        if (settleStart < 0) {
            settleStart = System.currentTimeMillis();
            return false;
        }
        return(System.currentTimeMillis() - settleStart) >= SETTLE_TIME;

        //return juggler.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor when done
        juggler.stop();
    }
}
