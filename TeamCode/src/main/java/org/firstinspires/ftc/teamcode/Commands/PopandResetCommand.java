package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;

public class PopandResetCommand extends CommandBase {

    private final Popper popper;


    public PopandResetCommand(Popper popper) {
        this.popper = popper;
        addRequirements(popper);
    }
    @Override
    public void execute() {
        popper.set(Popper.PopperState.KICK);
        new WaitCommand(500);
        popper.set(Popper.PopperState.RESET);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
