package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;

public class KickCommand extends CommandBase {

    private final Slide slide;
    private long startTime;
    private static final long MAX_DURATION_MS = 750;

    public KickCommand(Slide slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.setTargetTicks(Slide.KICK_POS);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {

        long elapsed = System.currentTimeMillis() - startTime;
        boolean atTarget = slide.atTarget();
        boolean timeout = elapsed > MAX_DURATION_MS;

        return (atTarget) || timeout;

//        boolean atTarget =  slide.atTarget();
//        boolean timeout = (System.currentTimeMillis() - startTime) > MAX_DURATION_MS;
//        return atTarget || timeout;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
