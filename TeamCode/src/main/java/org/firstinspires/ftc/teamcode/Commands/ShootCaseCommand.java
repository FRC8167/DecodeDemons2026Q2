package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShootCaseCommand extends SequentialCommandGroup {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;

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

        // Attempt to read the motif
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();

        // If motif is missing, schedule a jiggle to help the sensor
        if (motif == null || motif.length < 3) {

            // Retry reading after jiggle
            motif = vision.getLatchedMotif();
        }


        int attempts = 0;
        // Build case key safely
        String caseKey = buildCaseKey(motif, colorMatch);

//        while (attempts < 2 && caseKey.contains("U")) {
//
//                caseKey = "UNKNOWN";
//                new JiggleCommand(juggler).schedule();
//                attempts += 1;
//            }

        addCommands(buildSequence(caseKey));
    }

    private SequentialCommandGroup buildSequence(String key) {
        switch (key) {

            case "MPGPJGPP":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MPPGJGPP":
            case "MGPPJPPG":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MPPGJPGP":
            case "MPGPJPPG":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MGPPJPGP":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MPPGJPPG":
            case "MPGPJPGP":
            case "MGPPJGPP":
            case "UNKNOWN":
            default:
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new InstantCommand(shooter::stop)
                );
        }
    }

    private String buildCaseKey(
            ColorMatch.ArtifactColor[] motif,
            ColorMatch colorMatch

    ) {
        if (motif == null || motif.length < 3) {

            return "UNKNOWN";
        }

        StringBuilder sb = new StringBuilder("M");
        for (ColorMatch.ArtifactColor c : motif) {
            sb.append(colorToChar(c));
        }

        sb.append("J");
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0)));
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1)));
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2)));

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
