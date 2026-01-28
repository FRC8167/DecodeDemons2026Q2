package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;

    private final List<ColorMatch.ArtifactColor> motif = new ArrayList<>();
    private final ColorMatch.ArtifactColor[] virtualSlots = new ColorMatch.ArtifactColor[3];

    private int motifIndex = 0;
    private boolean shotInProgress = false;
    private final ElapsedTime kickTimer = new ElapsedTime();

    private boolean rotating = false;

    public ShootCaseCommand(Juggler juggler, Popper popper, Shooter shooter,
                            ColorMatch colorMatch, Vision vision) {
        this.juggler = juggler;
        this.popper = popper;
        this.shooter = shooter;
        this.colorMatch = colorMatch;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        // latch motif
        motif.clear();
        ColorMatch.ArtifactColor[] latched = vision.getLatchedMotif();
        if (latched != null && latched.length > 0) {
            motif.addAll(Arrays.asList(latched));
        }

        // latch initial slots
        virtualSlots[0] = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        virtualSlots[1] = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
        virtualSlots[2] = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);

        motifIndex = 0;
        shotInProgress = false;
        rotating = false;
    }

    @Override
    public void execute() {
        if (motifIndex >= 3) return; // max 3 shots per command

        ColorMatch.ArtifactColor target = motifIndex < motif.size() ? motif.get(motifIndex) : ColorMatch.ArtifactColor.UNKNOWN;

        // ROTATION LOGIC
        if (!rotating && !shotInProgress) {
            if (virtualSlots[1] == target) {
                rotating = true;
//                rotateTimer.reset();
                juggler.rotateOneSlot(Juggler.Direction.CW);
                // rotate virtual slots CW
                ColorMatch.ArtifactColor tmp = virtualSlots[0];
                virtualSlots[0] = virtualSlots[1];
                virtualSlots[1] = virtualSlots[2];
                virtualSlots[2] = tmp;
            } else if (virtualSlots[2] == target) {
                rotating = true;
//                rotateTimer.reset();
                juggler.rotateOneSlot(Juggler.Direction.CCW);
                // rotate virtual slots CCW
                ColorMatch.ArtifactColor tmp = virtualSlots[0];
                virtualSlots[0] = virtualSlots[2];
                virtualSlots[2] = virtualSlots[1];
                virtualSlots[1] = tmp;
            } else if (virtualSlots[0] == ColorMatch.ArtifactColor.NONE) {
                if (virtualSlots[1] != ColorMatch.ArtifactColor.NONE) {
                    rotating = true;
//                    rotateTimer.reset();
                    juggler.rotateOneSlot(Juggler.Direction.CW);
                    // rotate virtual slots CW
                    ColorMatch.ArtifactColor tmp = virtualSlots[0];
                    virtualSlots[0] = virtualSlots[1];
                    virtualSlots[1] = virtualSlots[2];
                    virtualSlots[2] = tmp;
                } else if (virtualSlots[2] != ColorMatch.ArtifactColor.NONE) {
                    rotating = true;
//                    rotateTimer.reset();
                    juggler.rotateOneSlot(Juggler.Direction.CCW);
                    // rotate virtual slots CCW
                    ColorMatch.ArtifactColor tmp = virtualSlots[0];
                    virtualSlots[0] = virtualSlots[2];
                    virtualSlots[2] = virtualSlots[1];
                    virtualSlots[1] = tmp;
                } else {
                    // Break and stop if no artifacts are present
                    motifIndex = 3;
                    return;
                }
            }
        }

        if (rotating && juggler.atTarget()) {
            rotating = false;
        }

        // SHOOT LOGIC
        if (!shotInProgress && !rotating) {
            // fire whatever is in slot0 (even UNKNOWN)
            shooter.smartVelocity(vision.getDistanceToGoal(), virtualSlots[0]);
            if (shooter.atTargetVelocity()) {
                popper.set(Popper.PopperState.KICK);
                kickTimer.reset();
                shotInProgress = true;
            }
        }

        // RESET POPPER & NEXT
        if (shotInProgress && kickTimer.milliseconds() > 250) {
            popper.set(Popper.PopperState.RESET);
            shotInProgress = false;
            motifIndex++;
            // change virtualSlots for artifact removed
            virtualSlots[0] = ColorMatch.ArtifactColor.NONE;
        }
    }

    @Override
    public boolean isFinished() {
        return motifIndex >= 3 && !shotInProgress && !rotating;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
