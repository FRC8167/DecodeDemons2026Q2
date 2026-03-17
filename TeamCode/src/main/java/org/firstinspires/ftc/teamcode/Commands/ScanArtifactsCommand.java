package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;


public class ScanArtifactsCommand extends CommandBase {

    private final ColorMatch colorMatch;
    private final JugglerAbsolute juggler;
    Robot robot = Robot.getInstance();

    public ScanArtifactsCommand(ColorMatch colorMatch, JugglerAbsolute juggler) {
        this.colorMatch = colorMatch;
        this.juggler = juggler;
    }

    @Override
    public void execute() {
        colorMatch.updateSpinStates(juggler.getSlotIndex());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
