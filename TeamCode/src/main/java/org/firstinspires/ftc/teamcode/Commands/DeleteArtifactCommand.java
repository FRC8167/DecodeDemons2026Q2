package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;
import org.firstinspires.ftc.teamcode.SubSystems.SpinStatesSingleton;


public class DeleteArtifactCommand extends CommandBase {

    private final JugglerAbsolute juggler;

    public DeleteArtifactCommand(JugglerAbsolute juggler) {
        this.juggler = juggler;
    }

    @Override
    public void execute() {
        SpinStatesSingleton.getInstance().deleteByJugglerIndex(juggler.getSlotIndex());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
