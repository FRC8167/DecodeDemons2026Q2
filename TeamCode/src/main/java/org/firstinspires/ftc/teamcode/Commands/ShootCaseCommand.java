package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
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
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
        String caseKey = buildCaseKey(motif, colorMatch);

        if (caseKey.contains("U")) {caseKey = "UNKNOWN";}
        sequence = buildSequence(caseKey);
        sequence.schedule();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private SequentialCommandGroup buildSequence(String key) {

        switch (key) {

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
                        // Spin shooter down shooter
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MPPGJPGP":
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        // Shoot then rotate CW and shoot repeated 2 times
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1),
                        new PopandResetCommand(popper),
                        // Spin shooter down shooter
                        new ShooterSpinUpCommand(shooter, 0.0)
                );

            case "MGPPJPGP":
                return new SequentialCommandGroup(
                        // Spin up shooter at start of sequence
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        // Rotate CW and shoot repeated 3 times
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        // Spin shooter down shooter
                        new ShooterSpinUpCommand(shooter, 0.0)
                );



            case "MPPGJPPG":
            case "MPGPJPGP":
            case "MGPPJGPP":
            case "UNKNOWN":
            default:
                return new SequentialCommandGroup(
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        // Shoot then rotate CW and shoot repeated 2 times
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new PopandResetCommand(popper),
                        // Spin shooter down shooter
                        new ShooterSpinUpCommand(shooter, 0.0)
                );
        }
    }




    private String buildCaseKey(
            ColorMatch.ArtifactColor[] motif,
            ColorMatch colorMatch
    ) {
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
