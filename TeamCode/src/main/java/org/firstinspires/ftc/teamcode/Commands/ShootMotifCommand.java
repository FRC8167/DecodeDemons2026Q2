package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShootMotifCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final ColorMatch colorMatch;
    private final Vision vision;

    private ColorMatch.ArtifactColor[] motif;
    private int shotIndex;
    private Integer targetSlot = null;
    private boolean rotating;
    private boolean popperKicked;

    private final ElapsedTime timer = new ElapsedTime();
    private static final long POP_TIME_MS = 500;



    public ShootMotifCommand(
            Juggler juggler,
            Popper popper,
            ColorMatch colorMatch,
            Vision vision
    ) {
        this.juggler = juggler;
        this.popper = popper;
        this.colorMatch = colorMatch;
        this.vision = vision;
        addRequirements(juggler, popper);
    }

    private String buildCaseKey(
            ColorMatch.ArtifactColor[] motif,
            ColorMatch colorMatch
    ) {
        StringBuilder sb = new StringBuilder("M");

        for (ColorMatch.ArtifactColor c : motif) {
            sb.append(c.name().charAt(0));
        }

        sb.append("J");

        sb.append(colorMatch.detectColor(ColorMatch.Slot.SLOT_0).name().charAt(0));
        sb.append(colorMatch.detectColor(ColorMatch.Slot.SLOT_1).name().charAt(0));
        sb.append(colorMatch.detectColor(ColorMatch.Slot.SLOT_2).name().charAt(0));

        return sb.toString();
    }


    @Override
    public void initialize() {

        motif = vision.getLatchedMotif();
        shotIndex = 0;

        rotating = false;
        popperKicked = false;
        targetSlot = null;   // ✅

        popper.set(Popper.PopperState.RESET);
        timer.reset();

        if (motif == null || motif.length == 0) {
            cancel();
        }
    }

    @Override
    public void execute() {

        // All shots complete
        if (shotIndex >= motif.length) return;

        ColorMatch.ArtifactColor targetColor = motif[shotIndex];

        // Find slot with desired color
        int slot = colorMatch.findSlotWithColor(targetColor);

        // Ball missing → skip this shot cleanly
        if (slot == -1) {
            shotIndex++;
            rotating = false;
            popperKicked = false;
            timer.reset();
            return;
        }

        // Command rotation ONCE
        if (!rotating) {
            juggler.rotateToSlot(slot);
            rotating = true;
        }

        // Wait until rotation finishes
        if (!juggler.atTarget()) return;

        // Kick exactly once
        if (!popperKicked) {
            popper.set(Popper.PopperState.KICK);
            popperKicked = true;
            timer.reset();
        }

        // Finish shot after pop time
        if (timer.milliseconds() >= POP_TIME_MS) {
            popper.set(Popper.PopperState.RESET);

            shotIndex++;
            rotating = false;
            popperKicked = false;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return shotIndex >= motif.length
                && popper.getPopperState() == Popper.PopperState.RESET;
    }

    @Override
    public void end(boolean interrupted) {
        popper.set(Popper.PopperState.RESET);
    }
}
