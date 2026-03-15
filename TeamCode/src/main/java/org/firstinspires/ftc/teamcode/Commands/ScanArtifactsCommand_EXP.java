package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Cogintilities.Color;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;


public class ScanArtifactsCommand_EXP extends CommandBase {

    private final ColorMatch colorMatch;
    private final JugglerAbsolute_EXP juggler;
    Robot robot = Robot.getInstance();

    public ScanArtifactsCommand_EXP(ColorMatch colorMatch, JugglerAbsolute_EXP juggler) {
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
