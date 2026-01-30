package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;
    // private SequentialCommandGroup sequence;
    // private SeqCmdAuto sequence;
    private SequentialCommandGroup sequence;

    public ShootCaseCommand(
            Juggler juggler,
            Popper popper,
            Shooter shooter,
            ColorMatch colorMatch,
            Vision vision) {
        this.juggler = juggler;
        this.popper = popper;
        this.shooter = shooter;
        this.colorMatch = colorMatch;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        super.initialize();
        // Attempt to read the motif
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();

        sequence = buildAutoSequence(motif, colorMatch);
        if (sequence == null) {
            sequence = new SequentialCommandGroup();
            // sequence = new SeqCmdAuto(caseKey, colorMatch);
        }
        sequence.initialize();
        // throw new RuntimeException("Motif: " + Arrays.toString(motif) +"\n"+"Key:
        // "+caseKey);

    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    private SequentialCommandGroup buildAutoSequence(ColorMatch.ArtifactColor[] motif, ColorMatch colorMatch) {
        if (motif == null || motif.length != 3) {
            // TODO: Handle this case
            // Set random motif
            motif = new ColorMatch.ArtifactColor[] {
                    ColorMatch.ArtifactColor.PURPLE,
                    ColorMatch.ArtifactColor.PURPLE,
                    ColorMatch.ArtifactColor.GREEN
            };
        }

        SequentialCommandGroup seq = new SequentialCommandGroup();

        // Track virtual slots: [Slot 0, Slot 1, Slot 2]
        ColorMatch.ArtifactColor[] virtualSlots = new ColorMatch.ArtifactColor[3];
        virtualSlots[0] = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        virtualSlots[1] = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
        virtualSlots[2] = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);

        // Count total artifacts initially
        int totalArtifacts = 0;
        int countedTotalArtifacts = 0;
        for (ColorMatch.ArtifactColor c : virtualSlots) {
            if (c != ColorMatch.ArtifactColor.NONE && c != ColorMatch.ArtifactColor.UNKNOWN) {
                countedTotalArtifacts++;
            }
        }

        CommandBase action = null;

        if(countedTotalArtifacts !=3){

            seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
            seq.addCommands(new PopandResetCommand(popper));
            seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));
            seq.addCommands(new PopandResetCommand(popper));
            seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));
            seq.addCommands(new PopandResetCommand(popper));
            seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));



            seq.addCommands(new InstantCommand(shooter::stop));
            return seq;
        }
        totalArtifacts=3;

        // If Slot 0 is unknown/empty initially, maybe rotate to fill it?
        // Logic below handles selection. If Slot 0 is UNKNOWN but needed, it might be
        // an issue,
        // but let's assume detection works.
        // Existing code had a check:
        // if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) ==
        // ColorMatch.ArtifactColor.UNKNOWN)
        // new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
        // This implies if S0 is empty, we rotate CCW 1 (Move S1->S0).
        // I will replicate this behavior as a pre-step if S0 is empty,
        // but only if we don't handle it in the loop logic.
        // Actually, let's trust the loop logic. If we need a color and it's at S1, we
        // will rotate CCW 1 regardless.

        for (ColorMatch.ArtifactColor targetColor : motif) {
            // Stop if we ran out of artifacts
            if (totalArtifacts <= 0) {
                break;
            }

            // Find index of targetColor in virtualSlots
            int targetIndex = -1;

            // Prefer Slot 0, then 1, then 2 (or based on rotation cost? 0 is best).
            if (virtualSlots[0] == targetColor)
                targetIndex = 0;
            else if (virtualSlots[1] == targetColor)
                targetIndex = 1;
            else if (virtualSlots[2] == targetColor)
                targetIndex = 2;

            if (targetIndex == -1) {
                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) != ColorMatch.ArtifactColor.NONE)
                    targetIndex =0;
                else if (colorMatch.detectColor(ColorMatch.Slot.SLOT_1) != ColorMatch.ArtifactColor.NONE)
                    targetIndex =1;
                else
                    targetIndex=2;
                // Color not found in slots!
                //continue;
            }

            //CommandBase action = null;

            if (targetIndex == 0) {
                // Already at 0. Just shoot.
                action = new ShooterSmartSpinUpCommand(shooter, vision);
            } else if (targetIndex == 1) {
                // At Slot 1. Rotate CCW 1 to bring S1 -> S0.
                action = new ParallelCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));

                // Update virtual slots: CCW 1
                // S0 <- S1
                // S1 <- S2
                // S2 <- S0 (old S0)
                ColorMatch.ArtifactColor temp = virtualSlots[0];
                virtualSlots[0] = virtualSlots[1];
                virtualSlots[1] = virtualSlots[2];
                virtualSlots[2] = temp;

            } else {
                // At Slot 2. Rotate CW 1 to bring S2 -> S0.
                action = new ParallelCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1));

                // Update virtual slots: CW 1
                // S0 <- S2
                // S2 <- S1
                // S1 <- S0 (old S0)
                ColorMatch.ArtifactColor temp = virtualSlots[0];
                virtualSlots[0] = virtualSlots[2];
                virtualSlots[2] = virtualSlots[1];
                virtualSlots[1] = temp;
            }

            seq.addCommands(action);
            seq.addCommands(new PopandResetCommand(popper));

            // Artifact at 0 is now used/popped.
            virtualSlots[0] = ColorMatch.ArtifactColor.NONE;
            totalArtifacts--;

            // Stop shooter at the very end?
            // The original code has `new InstantCommand(shooter::stop))` at the end of the
            // sequence.
        }

        seq.addCommands(new InstantCommand(shooter::stop));
        return seq;
    }

    private String buildCaseKey(
            ColorMatch.ArtifactColor[] motif,
            ColorMatch colorMatch

    ) {

        char Slot0_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
        char Slot1_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
        char Slot2_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
        if (motif == null || motif.length < 3 || Slot0_char == 'U' || Slot1_char == 'U' || Slot2_char == 'U') {

            return "UNKNOWN";
        }

        StringBuilder sb = new StringBuilder("M");
        for (ColorMatch.ArtifactColor c : motif) {
            sb.append(colorToChar(c));
        }

        sb.append("J");
        sb.append(Slot0_char);
        sb.append(Slot1_char);
        sb.append(Slot2_char);

        return sb.toString();
    }

    private char colorToChar(ColorMatch.ArtifactColor c) {
        switch (c) {
            case PURPLE:
                return 'P';
            case GREEN:
                return 'G';
            default:
                return 'U';
        }
    }

    // Are there 3 artifacts identified?
    private boolean hasThreeArtifacts(ColorMatch colorMatch) {

        char Slot0_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
        char Slot1_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
        char Slot2_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2));

        if ((Slot0_char == 'P' || Slot0_char == 'G') &&
                (Slot1_char == 'P' || Slot1_char == 'G') &&
                (Slot2_char == 'P' || Slot2_char == 'G'))
            return true;

        return false;

    }

}