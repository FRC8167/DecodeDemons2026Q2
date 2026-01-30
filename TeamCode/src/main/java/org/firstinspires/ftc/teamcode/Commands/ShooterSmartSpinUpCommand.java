package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShooterSmartSpinUpCommand extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;
    //private final ColorMatch colorMatch;


    public ShooterSmartSpinUpCommand(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        //this.colorMatch = colorMatch;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        //Read slot0 color and spin shooter accordingly
        //ColorMatch.ArtifactColor slot0Color = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        shooter.smartVelocity(vision.getDistanceToGoal());

    }

    @Override
    public boolean isFinished() {
        return shooter.atTargetVelocity();
    }

    @Override
    public void end(boolean interrupted) {
//            shooter.stop();

    }
}
