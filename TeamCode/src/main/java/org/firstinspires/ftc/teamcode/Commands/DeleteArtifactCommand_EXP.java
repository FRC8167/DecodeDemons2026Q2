package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;
import org.firstinspires.ftc.teamcode.SubSystems.SpinStatesSingleton_Eric;


public class DeleteArtifactCommand_EXP extends CommandBase {

    private final JugglerAbsolute_EXP juggler;

    public DeleteArtifactCommand_EXP(JugglerAbsolute_EXP juggler) {
        this.juggler = juggler;
    }

    @Override
    public void execute() {
        SpinStatesSingleton_Eric.getInstance().deleteByJugglerIndex(juggler.getSlotIndex());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
