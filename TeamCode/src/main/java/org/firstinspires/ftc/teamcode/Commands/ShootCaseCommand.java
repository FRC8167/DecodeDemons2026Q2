package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Cogintilities.State;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;
import org.firstinspires.ftc.teamcode.SubSystems.SpinStatesSingleton;

public class ShootCaseCommand extends SequentialCommandGroup {

    private final JugglerAbsolute juggler;
//    private final Popper popper;
    private final Shooter shooter;
    private final Slide slide;
    private final ColorMatch colorMatch;
    private final
    LimeLightVision vision;
    private SequentialCommandGroup sequence;
//    private final int scoredArtifacts;

    public ShootCaseCommand(
            JugglerAbsolute juggler,
//            Popper popper,
            Slide slide,
            Shooter shooter,
            ColorMatch colorMatch,
            LimeLightVision vision
//            int scoredArtifacts
    ) {
        this.juggler = juggler;
//        this.popper = popper;
        this.slide = slide;
        this.shooter = shooter;
        this.colorMatch = colorMatch;
        this.vision = vision;
//        this.scoredArtifacts = scoredArtifacts;
    }

    @Override
    public void initialize() {
        super.initialize();
        // Attempt to read the motif
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();

//        colorMatch.updateSpinStates(juggler.getSlotIndex());

        sequence = buildAutoSequence(motif);

        sequence.initialize();

    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    @NonNull
    private SequentialCommandGroup buildAutoSequence(ColorMatch.ArtifactColor[] motif) {

        if (motif == null || motif.length != 3) {
            // Set random motif
            motif = new ColorMatch.ArtifactColor[] {
                    ColorMatch.ArtifactColor.PURPLE,
                    ColorMatch.ArtifactColor.PURPLE,
                    ColorMatch.ArtifactColor.GREEN
            };
        }

        ColorMatch.ArtifactColor[] adjustedMotif = State.sequenceRevert(SpinStatesSingleton.getNextToShoot(Robot.getArtifactsScored(), State.sequenceMigrate(motif)));

        State[] sequence = State.sequenceMigrate(adjustedMotif);
        State[] bestToShoot = SpinStatesSingleton.getInstance().toBestStatesAvailable(sequence);

        SequentialCommandGroup seq = new SequentialCommandGroup();

        seq.addCommands(new ShooterSmartSpinUpCommand(shooter, vision));

        for (State state : bestToShoot) {
            seq.addCommands(new RotateToStateCommand(juggler, state));

            seq.addCommands(new WaitUntilCommand(shooter::atTargetVelocity));

            seq.addCommands(new KickCommand(slide));
            seq.addCommands(new NestCommand(slide));

            seq.addCommands(new DeleteArtifactCommand(juggler));

        }

        seq.addCommands(new InstantCommand(shooter::stop));
        return seq;
    }

}