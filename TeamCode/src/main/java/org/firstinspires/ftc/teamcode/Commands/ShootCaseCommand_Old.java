package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;

@Deprecated
public class ShootCaseCommand_Old extends SequentialCommandGroup {

    private final JugglerAbsolute juggler;
//    private final Popper popper;
    private final Shooter shooter;
    private final Slide slide;
    private final ColorMatch colorMatch;
    private final
    LimeLightVision vision;
    private SequentialCommandGroup sequence;

    public ShootCaseCommand_Old(
            JugglerAbsolute juggler,
//            Popper popper,
            Slide slide,
            Shooter shooter,
            ColorMatch colorMatch,
            LimeLightVision vision) {
        this.juggler = juggler;
//        this.popper = popper;
        this.slide = slide;
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
//        ColorMatch.ArtifactColor[] virtualSlots = new ColorMatch.ArtifactColor[3];
//        virtualSlots[0] = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
//        virtualSlots[1] = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
//        virtualSlots[2] = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);
        // 1. Initial Data Collection (Read once, then deduce)
        ColorMatch.ArtifactColor s0 = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        ColorMatch.ArtifactColor s1 = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
        ColorMatch.ArtifactColor s2 = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);

        ColorMatch.ArtifactColor[] deducedSlots = deduceMissingColors(s0, s1, s2);
        List<ColorMatch.ArtifactColor> virtualSlots = new ArrayList<>(Arrays.asList(deducedSlots));

        // Count total artifacts initially
        int totalArtifacts = 0;
        int countedTotalArtifacts = 0;

        for (ColorMatch.ArtifactColor c : virtualSlots) {
            if (c != ColorMatch.ArtifactColor.NONE && c != ColorMatch.ArtifactColor.UNKNOWN) {
                countedTotalArtifacts++;
            }
        }


        CommandBase action = null;
        //seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
//        seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));

        if(countedTotalArtifacts !=3){

            seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
//            seq.addCommands(new WaitCommand(550));
            seq.addCommands(new WaitUntilCommand(shooter::atTargetVelocity));
//            seq.addCommands(new PopandResetCommand(popper));
            seq.addCommands(new KickCommand(slide));
            seq.addCommands(new NestCommand(slide));
//            seq.addCommands(new WaitUntilCommand(slide::atTarget));
            seq.addCommands(new RotateXSlotsCommand(juggler, JugglerAbsolute.Direction.CW, 1));
//            seq.addCommands(new RotateOneSlotCommand(juggler, Juggler.Direction.CW));
//            seq.addCommands(new PopandResetCommand(popper));
//            seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
            seq.addCommands(new KickCommand(slide));
            seq.addCommands(new NestCommand(slide));
//            seq.addCommands(new WaitUntilCommand(slide::atTarget));
            seq.addCommands(new RotateXSlotsCommand(juggler, JugglerAbsolute.Direction.CW, 1));
//            seq.addCommands(new RotateOneSlotCommand(juggler, Juggler.Direction.CW));
//            seq.addCommands(new PopandResetCommand(popper));
//            seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
            seq.addCommands(new KickCommand(slide));
            seq.addCommands(new NestCommand(slide));
//            seq.addCommands(new WaitUntilCommand(slide::atTarget));
//            seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));
//            seq.addCommands(new RotateOneSlotCommand(juggler, Juggler.Direction.CW));
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
//        seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
//        seq.addCommands(new WaitCommand(250));
        for (ColorMatch.ArtifactColor targetColor : motif) {
            // Stop if we ran out of artifacts
            if (totalArtifacts <= 0) {
                break;
            }

            // Find index of targetColor in virtualSlots
            int targetIndex = -1;

            // Prefer Slot 0, then 1, then 2 (or based on rotation cost? 0 is best).
            targetIndex = virtualSlots.indexOf(targetColor);
//            if (virtualSlots[0] == targetColor)
//                targetIndex = 0;
//            else if (virtualSlots[1] == targetColor)
//                targetIndex = 1;
//            else if (virtualSlots[2] == targetColor)
//                targetIndex = 2;

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
            //action = new ShooterSmartSpinUpCommand(shooter, vision);
            if (targetIndex == 0) {
                // Already at 0. Just shoot.
                seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));

                //action = new ShooterSmartSpinUpCommand(shooter, vision);
            } else if (targetIndex == 1) {
//                action = new ShooterSmartSpinUpCommand(shooter, vision);
                // At Slot 1. Rotate CCW 1 to bring S1 -> S0.
//                action = new ParallelCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));
//                //already sping so shoot; distance should not be changing that much
//                action =  new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1);
                seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
                seq.addCommands(new RotateOneSlotCommand(juggler, JugglerAbsolute.Direction.CW));

                //seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));



                // Update virtual slots: CCW 1
                // S0 <- S1
                // S1 <- S2
                // S2 <- S0 (old S0)
                Collections.rotate(virtualSlots, -1);
//                ColorMatch.ArtifactColor temp = virtualSlots[0];
//                virtualSlots[0] = virtualSlots[1];
//                virtualSlots[1] = virtualSlots[2];
//                virtualSlots[2] = temp;

            } else {
                // At Slot 2. Rotate CW 1 to bring S2 -> S0.
                //already sping so shoot; distance should not be changing that much
                seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));
                seq.addCommands(new RotateOneSlotCommand(juggler, JugglerAbsolute.Direction.CCW));
                //seq.addCommands(new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1));
                //action =  new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);

                // Update virtual slots: CW 1
                // S0 <- S2
                // S2 <- S1
                // S1 <- S0 (old S0)
                Collections.rotate(virtualSlots, 1);
//                ColorMatch.ArtifactColor temp = virtualSlots[0];
//                virtualSlots[0] = virtualSlots[2];
//                virtualSlots[2] = virtualSlots[1];
//                virtualSlots[1] = temp;
            }

            //seq.addCommands(action);
            seq.addCommands(
//                    new WaitUntilCommand(juggler::isReadyToFire),
                    new WaitUntilCommand(shooter::atTargetVelocity),
                    new KickCommand(slide),
                    new NestCommand(slide)
            );
//            seq.addCommands(new KickCommand(slide));
//            seq.addCommands(new NestCommand(slide));
//            seq.addCommands(new PopandResetCommand(popper));

            // Artifact at 0 is now used/popped.
            // Mark slot as empty
            virtualSlots.set(0, ColorMatch.ArtifactColor.NONE);
            //virtualSlots[0] = ColorMatch.ArtifactColor.NONE;
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

    //===========================

    private ColorMatch.ArtifactColor[] deduceMissingColors(ColorMatch.ArtifactColor s0, ColorMatch.ArtifactColor s1, ColorMatch.ArtifactColor s2) {
        ColorMatch.ArtifactColor[] slots = {s0, s1, s2};
        int purples = 0, greens = 0;
        int unknownCount = 0;
        int lastUnknownIdx = -1;
        int knownIdx=0;

        for (int i = 0; i < 3; i++) {
            if (slots[i] == ColorMatch.ArtifactColor.PURPLE)
                purples++;
            else if (slots[i] == ColorMatch.ArtifactColor.GREEN)
                greens++;
            else {
                unknownCount++;
                lastUnknownIdx = i;
            }
        }

        // If exactly one is unknown, we can deduce it with 100% certainty
        if (unknownCount == 1) {
            if (purples == 2) slots[lastUnknownIdx] = ColorMatch.ArtifactColor.GREEN;
            else if (greens == 1) slots[lastUnknownIdx] = ColorMatch.ArtifactColor.PURPLE;
        }
        // If two are unknown, we fill based on the remaining pool
        else if (unknownCount == 2) {
            // Find which slot IS known
            for (int i = 0; i < 3; i++) {
                if ((slots[i] == ColorMatch.ArtifactColor.PURPLE) || (slots[i] == ColorMatch.ArtifactColor.GREEN))
                    knownIdx = i;
            }
            // Fill the others with the remaining pieces of the 2P/1G set
            if (slots[knownIdx] == ColorMatch.ArtifactColor.GREEN) {
                for(int i=0; i<3; i++) if(i != knownIdx) slots[i] = ColorMatch.ArtifactColor.PURPLE;
            } else {
                // One purple known, so one purple and one green remain
                // (Order is a guess here, but better than skipping)
                boolean greenAssigned = false;
                for(int i=0; i<3; i++) {
                    if(i != knownIdx) {
                        slots[i] = greenAssigned ? ColorMatch.ArtifactColor.PURPLE : ColorMatch.ArtifactColor.GREEN;
                        greenAssigned = true;
                    }
                }
            }
        }
        return slots;
    }

}