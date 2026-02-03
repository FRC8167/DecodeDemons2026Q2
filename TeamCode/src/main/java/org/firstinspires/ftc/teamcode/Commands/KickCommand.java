package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;

public class KickCommand extends CommandBase {

    private final Slide slide;

    public KickCommand(Slide slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.setTargetTicks(Slide.KICK_POS);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return slide.atTarget();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
