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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;
    private SequentialCommandGroup sequence;
    //create a list from the color array of available options
    private final List<ColorMatch.ArtifactColor> remainingMotif = new ArrayList<>();

    public ShootCaseCommand(
            Juggler juggler,
            Popper popper,
            Shooter shooter,
            ColorMatch colorMatch,
            Vision vision
    ) {
        this.juggler = juggler;
        this.popper = popper;
        this.shooter = shooter;
        this.colorMatch = colorMatch;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        // latch the motif from vision
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
        if (motif != null && motif.length > 0) {
            remainingMotif.addAll(Arrays.asList(motif));
        }
    }

    @Override
    public void execute() {
        if (remainingMotif.isEmpty()) return; //no artifacts to shoot

        //read the next color in the motif pattern
        ColorMatch.ArtifactColor target = remainingMotif.get(0);

        // Read the slots at the start of the ShootCaseCommand
        ColorMatch.ArtifactColor s0 = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        ColorMatch.ArtifactColor s1 = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
        ColorMatch.ArtifactColor s2 = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);

        // Dynamically (right word?) create the proper order of events
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        if (s0 == target) {
            // If the target motif color is in slot0, spin up shooter, popand release, remove from motif list
            sequence.addCommands(
                    new ShooterSmartSpinUpCommand(shooter, vision),
                    new PopandResetCommand(popper),
                    new InstantCommand(() -> remainingMotif.remove(0))
            );

        } else if (s1 == target) {
            // If the target motif color is in slot1, rotate 1 slot CW to move to slot0
            sequence.addCommands(
                    new RotateOneSlotCommand(juggler, Juggler.Direction.CW)
            );

        } else if (s2 == target) {
            // If the target motif color is in slot2, rotate 1 slot CCW to move to slot0
            sequence.addCommands(
                    new RotateOneSlotCommand(juggler, Juggler.Direction.CCW)
            );

        } else {
            if (s0 == ColorMatch.ArtifactColor.UNKNOWN ||
                    s1 == ColorMatch.ArtifactColor.UNKNOWN ||
                    s2 == ColorMatch.ArtifactColor.UNKNOWN) {

                // If target is not visible (UNKNOWN), jiggle the juggler
                sequence.addCommands(new InstantCommand(() -> new JiggleCommand(juggler))
                );
            }

        }
    }



    @Override
    public boolean isFinished() {
        return remainingMotif.isEmpty();  //when no artifacts left to shoot
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

//    @Override
//    public void initialize() {
//        super.initialize();
//        // Attempt to read the motif
//        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
////        assert motif != null;  //this was causing the crash!
//
////        // If motif is missing, schedule a jiggle to help the sensor
////        if (motif == null || motif.length < 3) {
////
////            // Retry reading after jiggle
////            motif = vision.getLatchedMotif();
////        }
////
////
////        int attempts = 0;
//        // Build case key safely
//        String caseKey = buildCaseKey(motif, colorMatch);
//
////        while (attempts < 2 && caseKey.contains("U")) {
////
////                caseKey = "UNKNOWN";
////                new JiggleCommand(juggler).schedule();
////                attempts += 1;
////            }}
//        sequence = buildSequence(caseKey);
//        if (sequence == null) {
//            sequence = new SequentialCommandGroup();
//        }
//        sequence.initialize();
////        throw new RuntimeException("Motif: " + Arrays.toString(motif) +"\n"+"Key: "+caseKey);
//
//    }
//
//    @Override
//    public void execute() {
//        sequence.execute();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return sequence.isFinished();
//    }
//
//    private void SeqCmdAuto(String key, ColorMatch colorMatch){
//
//       colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
//
//    }
//    private SequentialCommandGroup buildSequence(String key) {
//
//        if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//
//        switch (key) {
//
//            case "MPGPJGPP":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                                new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJGPP":
//            case "MGPPJPPG":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                             new ShooterSmartSpinUpCommand(shooter, vision),
//                             new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJPGP":
//            case "MPGPJPPG":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MGPPJPGP":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJPPG":
//            case "MPGPJPGP":
//            case "MGPPJGPP":
//            case "MPPGJGGG":
//            case "MPGPJGGG":
//            case "MGPPJGGG":
//            case "UNKNOWN":
//            default:
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//                return new SequentialCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
////                        new WaitCommand(500),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                                new ShooterSmartSpinUpCommand(shooter, vision),
//                                new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper)
////                        new InstantCommand(shooter::stop)
//                );
//        }
//    }
//
//    private String buildCaseKey(
//            ColorMatch.ArtifactColor[] motif,
//            ColorMatch colorMatch
//
//    ) {
//
//        char Slot0_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        char Slot1_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        char Slot2_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//        if (motif == null || motif.length < 3 || Slot0_char == 'U' || Slot1_char == 'U' || Slot2_char == 'U') {
//
//            return "UNKNOWN";
//        }
//
//        StringBuilder sb = new StringBuilder("M");
//        for (ColorMatch.ArtifactColor c : motif) {
//            sb.append(colorToChar(c));
//        }
//
//        sb.append("J");
//        sb.append(Slot0_char);
//        sb.append(Slot1_char);
//        sb.append(Slot2_char);
//
//        return sb.toString();
//    }
//
//    private char colorToChar(ColorMatch.ArtifactColor c) {
//        switch (c) {
//            case PURPLE: return 'P';
//            case GREEN:  return 'G';
//            default:     return 'U';
//        }
//    }
//

}
