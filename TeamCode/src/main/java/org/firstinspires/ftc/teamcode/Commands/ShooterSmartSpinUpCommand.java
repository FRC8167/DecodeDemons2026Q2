package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterSmartSpinUpCommand extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;

    public ShooterSmartSpinUpCommand(Shooter shooterSubsystem, Vision vision) {
        this.shooter = shooterSubsystem;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.
                smartVelocity(vision.getDistanceToGoal());
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
