package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShooterSmartSpinUpCommand extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;
    private final ColorMatch colorMatch;


    public ShooterSmartSpinUpCommand(Shooter shooterSubsystem, Vision vision, ColorMatch.ArtifactColor colorMatch) {
        this.shooter = shooterSubsystem;
        this.vision = vision;
        this.colorMatch = colorMatch;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        ColorMatch.ArtifactColor currentColor = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        shooter.smartVelocity(vision.getDistanceToGoal(), currentColor);
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
