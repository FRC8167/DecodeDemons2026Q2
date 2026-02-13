package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;

public class NestCommand extends CommandBase {

    private final Slide slide;
    private long startTime;
    private static final long MAX_DURATION_MS = 500;

    public NestCommand(Slide slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.setTargetTicks(Slide.NEST_POS);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean atTarget =  slide.atTarget();
        boolean timeout = (System.currentTimeMillis() - startTime) > MAX_DURATION_MS;
        return atTarget || timeout;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
