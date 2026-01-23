package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.Arrays;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;
    private SequentialCommandGroup sequence;

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
        super.initialize();
        // Attempt to read the motif
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
//        assert motif != null;  //this was causing the crash!

//        // If motif is missing, schedule a jiggle to help the sensor
//        if (motif == null || motif.length < 3) {
//
//            // Retry reading after jiggle
//            motif = vision.getLatchedMotif();
//        }
//
//
//        int attempts = 0;
        // Build case key safely
        String caseKey = buildCaseKey(motif, colorMatch);

//        while (attempts < 2 && caseKey.contains("U")) {
//
//                caseKey = "UNKNOWN";
//                new JiggleCommand(juggler).schedule();
//                attempts += 1;
//            }}
        sequence = buildSequence(caseKey);
        if (sequence == null) {
            sequence = new SequentialCommandGroup();
        }
        sequence.initialize();
//        throw new RuntimeException("Motif: " + Arrays.toString(motif) +"\n"+"Key: "+caseKey);

    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    private SequentialCommandGroup buildSequence(String key) {

        switch (key) {

            case "MPGPJGPP":
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                                new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );

            case "MPPGJGPP":
            case "MGPPJPPG":
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                             new ShooterSmartSpinUpCommand(shooter, vision),
                             new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );

            case "MPPGJPGP":
            case "MPGPJPPG":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );

            case "MGPPJPGP":
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );

            case "MPPGJPPG":
            case "MPGPJPGP":
            case "MGPPJGPP":
            case "MPPGJGGG":
            case "MPGPJGGG":
            case "MGPPJGGG":
            case "UNKNOWN":
            default:
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                            new ShooterSmartSpinUpCommand(shooter, vision),
                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
//                        new WaitCommand(500),
                        new PopandResetCommand(popper),
                        new ParallelCommandGroup(
                                new ShooterSmartSpinUpCommand(shooter, vision),
                                new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
                        ),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );
        }
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
            case PURPLE: return 'P';
            case GREEN:  return 'G';
            default:     return 'U';
        }
    }


}
